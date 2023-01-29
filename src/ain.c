/*
 * ain - analog input data acq with compute functions run after each acq
 */

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sched.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <pthread.h>
#include <mqueue.h>
#include <inttypes.h>
#include <signal.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <sys/time.h>
#include <mqueue.h>
#include <errno.h>
#include <getopt.h>
#include <math.h>

#include <dlfcn.h>
#include <tcl.h>

#include <datapoint.h>
#include <qpcs_time.h>
#include <dservapi.h>
#include "ain_process.h"
#include "daemonizer.h"

#define DSERV_MQUEUE_NAME "/dserv"

static void timer_tick(int sig);

static void pabort(const char *s)
{
  perror(s);
  abort();
}


/*
 * processFunctions can be loaded from shared objects 
 */
typedef struct processFunctionInfo_s {
  char *name;
  void *handle;
  PROCESS_FUNC pfunc;
  PROCESS_NEWPARAM_FUNC newparamfunc;
  PROCESS_FREEPARAM_FUNC freeparamfunc;
  PROCESS_SETPARAM_FUNC setparamfunc;
  PROCESS_SETPARAM_FUNC getparamfunc;
} processFunctionInfo_t;

/* Store table of possible process functions here */
Tcl_HashTable processFunctionTable;

/*
 * Define a structure and variables for storing the data that
 * is received. When clients write data to us, we store it here.
 * When clients do reads, we get the data from here. Result: a
 * simple message queue.
 */
typedef struct item_s {
    struct item_s   *next;
    char            *data;
} item_t;

typedef struct adc_info_s {
  uint64_t timestamp;
  uint64_t last_timestamp;
  uint64_t elapsed;

  int fd;
  int device_id;               // file handle to spidev
  uint8_t mode;
  uint8_t bits_per_word;
  uint32_t speed_hz;
  uint16_t delay_usecs;

  int n_channels;
  uint16_t vals[4];
  int show_every;
  int log_every;
  int use_dserv;
  int dserv_mqueue;
  ds_datapoint_t *adc_dpoint;
  uint8_t resolution;           /* used to handle inversion   */
  uint8_t inversion_flag;	/* lower 4 bits control inv.  */
  Tcl_Interp *interp;
} adc_info_t;

adc_info_t *adc_info_ptr;	/* global available to timer  */

#ifdef __USAGE
%C - Driver for Microchip MCP320X A/D

Usage:
%C [Options...]

Options:
	-P prio           use this priority for the server (16)
	-n nchannels      number of channels to scan (2)
	-c script	  configure script to source (None)
	-i interval_ms    interval between reads (5ms)
	-s show_every     show_every n samples (0)
	-S Speed          speed for SPI communications (4000000)
	-C spiChannel     channel (1 -> /dev/spi1)
	-I spiId          SPI device ID (SPI_DEFAULT_ID)
	-l log_every      log every n samples to pubsub server
	-D use_dserv      log to dserv
        -p pubsubserver   use persistent PubSub server to use (/pps)
	-d                run in background (no)
	-v                increment verbosity level
#endif


static void spi_xchange(adc_info_t *adc, uint8_t tx[],  uint8_t rx[], int n)
{
  int ret;
  struct spi_ioc_transfer tr = {
      .tx_buf = (unsigned long)tx,
      .rx_buf = (unsigned long)rx,
      .len = n,
      .delay_usecs = adc->delay_usecs,
      .speed_hz = adc->speed_hz,
      .bits_per_word = adc->bits_per_word,
  };

  ret = ioctl(adc->fd, SPI_IOC_MESSAGE(1), &tr);
  if (ret < 1)
    pabort("can't send spi message");
}

void read_adc(adc_info_t *adcinfo)
{
  static unsigned char send[3], receive[3];
  int i;
  
  memset(adcinfo->vals, 0, 4*sizeof(uint16_t));
  for (i = 0; i < adcinfo->n_channels; i++) {
    send[0] = 0b00000110; // The Start Bit followed by SE
    send[1] = i<<6; // The MSB is the Single/Diff bit and it is followed by 000 for CH0
    send[2] = 0; // cThis byte doesn't need to be set, just for a clear display
    receive[0] = 0;
    spi_xchange(adcinfo, send, receive, 3);
    adcinfo->vals[i] = (short)(receive[1]&0b00001111)<<8|(short)receive[2];
  }
}


Tcl_Interp *init_tcl(void)
{
  /* Create a Tcl_Interp for threads to use */
  Tcl_Interp *interp = Tcl_CreateInterp();
  
  if (!interp) {
    perror("error creating TclInterp");
    return NULL;
  }

  if (Tcl_Init(interp) == TCL_ERROR) {
    return NULL;
  }

  return interp;
}

#if 0
static int setProcess(Tcl_Interp *interp, char *pname)
{
  processFunctionInfo_t *pinfo;
  Tcl_HashEntry *entryPtr;

  entryPtr = Tcl_FindHashEntry(&processFunctionTable, pname);
  if (!entryPtr) {
    Tcl_AppendResult(interp, "no process function named \"", pname,
		     "\"", NULL);
    return TCL_ERROR;
  }
  pinfo = Tcl_GetHashValue(entryPtr);
  if (!pinfo) {
	  Tcl_AppendResult(interp, "no process info set for \"", pname,
			   "\"", NULL);
	  return TCL_ERROR;
  }
#if 0
  if (ocb->process_params)
    ocb->free_params(ocb->process_params);

  ocb->process = pinfo->pfunc;
  ocb->process_setparam = pinfo->setparamfunc;
  ocb->process_getparam = pinfo->getparamfunc;
  ocb->process_params = pinfo->newparamfunc();
  ocb->free_params = pinfo->freeparamfunc;
#endif
  return TCL_OK;
}

static int setProcessParam(adc_info_t *adcinfo,
			   char *pname, char *pval,
			   int index)
{
  Tcl_Interp *interp = adcinfo->interp;
  int rval;
  Tcl_Obj *result;
  ain_param_setting_t psetting;
  
  if (!ocb->process || !ocb->process_params || !ocb->process_setparam) {
    Tcl_AppendResult(interp, "no process set on this channel", NULL);
    return TCL_ERROR;
  }

  psetting.pval = &pval;
  psetting.index = index;
  psetting.params = (void *) ocb->process_params;
  psetting.pname = pname;

  rval = ocb->process_setparam(&psetting);
  if (rval == AIN_PROCESS_DSERV) {
    dserv_set(&dattr->adc_info->ds_server, psetting.dpoint);
  }
  
  result = Tcl_NewIntObj(rval);
  Tcl_SetObjResult(interp, result);
  return TCL_OK;
}

static int getProcessParam(device_attr_t *dattr,
			   char *pname, int index, struct ocb *ocb)
{
  Tcl_Interp *interp = dattr->interp;
  int rval;
  ain_param_setting_t psetting;
  char *pval;
  
  if (!ocb->process || !ocb->process_params || !ocb->process_getparam) {
    Tcl_AppendResult(interp, "no process set on this channel", NULL);
    return TCL_ERROR;
  }

  psetting.pval = &pval;
  psetting.index = index;
  psetting.params = (void *) ocb->process_params;
  psetting.pname = pname;

  rval = ocb->process_getparam(&psetting);
  if (!rval)
    return TCL_ERROR;
  
  Tcl_SetObjResult(interp, Tcl_NewStringObj(pval, -1));
  return TCL_OK;
}
#endif

static int invertChannelsCmd (ClientData data, Tcl_Interp *interp,
			      int objc, Tcl_Obj *objv[])
{
  int flag, old_flag;
  
  adc_info_t *adcinfo = (adc_info_t *) data;

  if (objc != 2) {
    Tcl_WrongNumArgs(interp, 1, objv, "inversion_flag");
    return TCL_ERROR;
  }
  if (Tcl_GetIntFromObj(interp, objv[1], &flag) != TCL_OK) {
    return TCL_ERROR;
  }
  old_flag = adcinfo->inversion_flag;
  adcinfo->inversion_flag = flag;
  Tcl_SetObjResult(interp, Tcl_NewIntObj(old_flag));
  return TCL_OK;
}

static int loadProcessCmd (ClientData data, Tcl_Interp *interp,
			   int objc, Tcl_Obj *objv[])
{
  
  void *handle;
  PROCESS_FUNC pfunc;
  PROCESS_NEWPARAM_FUNC newpfunc;
  PROCESS_FREEPARAM_FUNC freepfunc;
  PROCESS_SETPARAM_FUNC setpfunc;
  PROCESS_SETPARAM_FUNC getpfunc;
  processFunctionInfo_t *pinfo;
  Tcl_HashEntry *entryPtr;
  int new;
  char *key = Tcl_GetStringFromObj(objv[2], NULL);
  if (objc != 3) {
    Tcl_WrongNumArgs(interp, 1, objv, "shared_object name");
    return TCL_ERROR;
  }

  entryPtr = Tcl_CreateHashEntry(&processFunctionTable,
				 key, &new);
  if (!new) {
    Tcl_AppendResult(interp, "process function ",
		     Tcl_GetStringFromObj(objv[1], NULL),
		     " already exists", (char *) NULL);
    return TCL_ERROR;
  }
  
  /* Open a shared library. */
  handle = dlopen( Tcl_GetStringFromObj(objv[1], NULL),  RTLD_NOW);
  if (!handle) {
    Tcl_AppendResult(interp, "error loading shared object \"",
		     Tcl_GetStringFromObj(objv[1], NULL),
		     "\"", (char *) NULL);
    return TCL_ERROR;

  }

  /* Find the address of the process function */
  pfunc = (PROCESS_FUNC) dlsym( handle, "onProcess" );
  if (!pfunc) {
    Tcl_AppendResult(interp, "onProcess not found in file \"",
		     Tcl_GetStringFromObj(objv[1], NULL),
		     "\"", (char *) NULL);
    dlclose(handle);
    return TCL_ERROR;
  }

  /* Find the address of param creation function */
  newpfunc = (PROCESS_NEWPARAM_FUNC) dlsym( handle, "newProcessParams" );
  if (!newpfunc) {
    Tcl_AppendResult(interp, "newProcessParams not found in file \"",
		     Tcl_GetStringFromObj(objv[1], NULL),
		     "\"", (char *) NULL);
    dlclose(handle);
    return TCL_ERROR;
  }

  freepfunc = (PROCESS_FREEPARAM_FUNC) dlsym( handle, "freeProcessParams" );
  if (!freepfunc) {
    Tcl_AppendResult(interp, "freeProcessParams not found in file \"",
		     Tcl_GetStringFromObj(objv[1], NULL),
		     "\"", (char *) NULL);
    dlclose(handle);
    return TCL_ERROR;
  }
  
  /* Find the address of param setting function */
  setpfunc = (PROCESS_SETPARAM_FUNC) dlsym( handle, "setProcessParams" );
  if (!setpfunc) {
    Tcl_AppendResult(interp, "setProcessParams not found in file \"",
		     Tcl_GetStringFromObj(objv[1], NULL),
		     "\"", (char *) NULL);
    dlclose(handle);
    return TCL_ERROR;
  }

  /* Find the address of param setting function */
  getpfunc = (PROCESS_SETPARAM_FUNC) dlsym( handle, "getProcessParams" );
  if (!getpfunc) {
    Tcl_AppendResult(interp, "getProcessParams not found in file \"",
		     Tcl_GetStringFromObj(objv[1], NULL),
		     "\"", (char *) NULL);
    dlclose(handle);
    return TCL_ERROR;
  }
  
  pinfo = ckalloc(sizeof(processFunctionInfo_t));
  if (!pinfo) {
    Tcl_AppendResult(interp, "error allocating storage for process function",
		     (char *) NULL);
    dlclose(handle);
    return TCL_ERROR;
  }

  pinfo->handle = handle;
  pinfo->name = strdup(Tcl_GetStringFromObj(objv[1], NULL));
  pinfo->pfunc = pfunc;
  pinfo->newparamfunc = newpfunc;
  pinfo->freeparamfunc = freepfunc;
  pinfo->setparamfunc = setpfunc;
  pinfo->getparamfunc = getpfunc;

  Tcl_SetHashValue(entryPtr, pinfo);

  return TCL_OK;
}

void process_and_notify(adc_info_t *adc_info)
{
#if 0  
  device_attr_t *dattr = adc_info->dattr;
  struct ocb *ocb;
  int rc;
  item_t *newitem;
  process_info_t pinfo;

  /* These are general for every process */
  pinfo.nchannels = adc_info->n_channels;
  pinfo.v = adc_info->vals;
  pinfo.timestamp = adc_info->timestamp;
  
  for (ocb = dattr->list; ocb; ocb = ocb->next) {

    if (!ocb->process) continue;

    rc = ocb->process(&pinfo, ocb->process_params);
    
    /* if rc is 0, then this client passes */
    if (rc == AIN_PROCESS_IGNORE)
      continue;

    if (rc == AIN_PROCESS_DSERV) {
      dserv_set(&adc_info->ds_server, pinfo.dpoint);
      continue;
    }

    // event should notify, but not listening
    if (!(ocb->deliverEvent == QPCS_AIN_PROCESSED || ocb->notify))
      continue;
    
    if (ocb->nitems == ocb->maxitems) {
      ocb->overflow = 1;
      continue;
    }
    if ((newitem = malloc(sizeof(item_t))) == NULL) return;

    /* 
     * each processor should have allocated the buffer
     * we just stuff into item 
     */
    newitem->data = *pinfo.result_str;

    if (ocb->firstitem)
      newitem->next = ocb->firstitem;
    else
      newitem->next = NULL;
    ocb->firstitem = newitem;
    ocb->nitems++;
    /* notify this specific client */
    if (ocb->notify) {
    	iofunc_notify_trigger_strict(&ocb->ctp, dattr->notify, 1,
				     IOFUNC_NOTIFY_INPUT);
    }
    if (ocb->deliverEvent == QPCS_AIN_PROCESSED)
      MsgDeliverEvent(ocb->rcvid, &ocb->event);
  }
#endif
  return;
}

static int timer_setup(adc_info_t *adc_info, int interval_ms)
{
  struct itimerval new_timer;
  struct itimerval old_timer;
  
  new_timer.it_value.tv_sec = 0;
  new_timer.it_value.tv_usec = interval_ms*1000;
  new_timer.it_interval.tv_sec = 0;
  new_timer.it_interval.tv_usec = interval_ms*1000;
  
  setitimer(ITIMER_REAL, &new_timer, &old_timer);
  signal(SIGALRM, timer_tick);

  return 0;
}

static void timer_tick(int sig)
{
  sig = sig;

  adc_info_t *dinfo = adc_info_ptr;

  static int count = 0;
  uint64_t timestamp;
  uint16_t maxval = (1 << dinfo->resolution)-1;
  
  read_adc(dinfo);
  
  if (dinfo->inversion_flag & 0x01)
    dinfo->vals[0] = maxval-dinfo->vals[0];
  if (dinfo->inversion_flag & 0x02)
    dinfo->vals[1] = maxval-dinfo->vals[1];
  if (dinfo->inversion_flag & 0x04)
    dinfo->vals[2] = maxval-dinfo->vals[2];
  if (dinfo->inversion_flag & 0x08)
    dinfo->vals[3] = maxval-dinfo->vals[3];
  
  timestamp = qpcs_now();
  dinfo->elapsed = timestamp-dinfo->timestamp;
  dinfo->last_timestamp = dinfo->timestamp;
  dinfo->timestamp = timestamp;
  
  if (!count) dinfo->last_timestamp = dinfo->timestamp;
  count++;

  if (dinfo->show_every && (count % dinfo->show_every == 0)) {
    printf("%8llu: %d %d %d %d\n",
	   dinfo->elapsed,
	   dinfo->vals[0], dinfo->vals[1],
	   dinfo->vals[2], dinfo->vals[3]);
  }
  
#if 0
  /* See if associated processor raises an event to forward */
  process_and_notify(dinfo);

  /* On every tick, deliver events to interested clients */
  deliver_raw_events(dinfo);
#endif

  if (dinfo->use_dserv && dinfo->log_every && (count % dinfo->log_every == 0)) {
    memcpy(dinfo->adc_dpoint->data.buf, dinfo->vals, 4);
    dinfo->adc_dpoint->timestamp = qpcs_now();
    
    static char buf[DPOINT_BINARY_FIXED_LENGTH];
    int size = DPOINT_BINARY_FIXED_LENGTH-1;
    buf[0] = DPOINT_BINARY_MSG_CHAR;
    dpoint_to_binary(dinfo->adc_dpoint, &buf[1], &size);
      
    if (mq_send(dinfo->dserv_mqueue, buf, sizeof(buf), 0) < 0) {
	fprintf(stderr, "error writing to mqueue\n");
    }
  }

}


int main(int argc, char *argv[])
{
  adc_info_t adc_info;		/* hold on to variables to share with acquisition function */
  int n_channels = 2;

  char spi_device[32];
  
  int         opt;
  int         priority = 16;  // default priority:default 16
  int         verbose = 0;
  int         daemon = 0;
  uint64_t    show_every = 0;
  char        *config_script = NULL;
  /* provide digital output on each scan */
  int         toggle_output_pin = 0;

  int         log_every = 0;
  int         use_dserv = 0;
  int         interval_ms = 1000;
  struct stat stat_buf;
    
  int speed = 10000000;
  int spi_device_id = 0;
  int spi_channel = 0;
  int mode = 4;

  int pathID;
  int ret;

  char *dev_name = "/dev/ain";
  char *ain_dpoint_name = "ain/vals";
  
  /* Process command line options */
  while ((opt = getopt(argc, argv, "N:V:n:p:I:S:l:i:P:s:C:c:vdD")) != -1) {
    switch (opt) {
    case 'P':   // priority
      priority = strtoul(optarg, NULL, 0) ;
      break;
    case 'i':   // kick interval
      interval_ms = strtoul(optarg, NULL, 0);
      break;
    case 'd':
      daemon++;
      break;
    case 's':
      show_every = strtoul(optarg, NULL, 0);
      break;
    case 'S':
      speed = strtoul(optarg, NULL, 0);
      break;
    case 'I':
      spi_device_id = strtoul(optarg, NULL, 0);
      break;
    case 'l':
      log_every = strtoul(optarg, NULL, 0);
      break;
    case 'D':
      use_dserv = 1;
      break;
    case 'V':
      ain_dpoint_name = optarg;
      break;
    case 'C':
      spi_channel = strtoul(optarg, NULL, 0);
      break;
    case 'N':
      dev_name = optarg;
      break;
    case 'c':
      config_script = optarg;
      break;
    case 'o':
      toggle_output_pin =  strtoul(optarg, NULL, 0);
      if (toggle_output_pin > 255) {
	printf("output pin for toggle out of range\n");
	exit(EXIT_FAILURE);
      }
      break;
    case 'n':
      n_channels =  strtoul(optarg, NULL, 0);
      if (n_channels < 1 || n_channels > 4) {
	printf("bad number of channels specified\n");
	exit(EXIT_FAILURE);
      }
      break;
    case 'v':   // verbose flag
      verbose++;
      break;
    }
  }

  /* set global pointer so timer can access */
  adc_info_ptr = &adc_info;
  
  /* This will be attached to the timer pulse callback */
  adc_info.n_channels = n_channels;
  adc_info.log_every = log_every;
  adc_info.show_every = show_every;
  adc_info.use_dserv = use_dserv;

  adc_info.device_id = spi_device_id;
  adc_info.speed_hz = speed;
  adc_info.delay_usecs = 0;
  adc_info.bits_per_word = 8;
  adc_info.mode = mode;

  // For inverting channels
  adc_info.resolution = 12;	/* adc range is 0-4096              */
  adc_info.inversion_flag = 0;	/* flip corresponding bit to invert */
  
  adc_info.timestamp = qpcs_now();

  
  snprintf(spi_device, 31, "/dev/spidev%d.%d", spi_channel, spi_device_id);

  adc_info.fd = open(spi_device, O_RDWR);
  if (adc_info.fd < 0) {
      pabort("can't open device");
  }

  /*
   * spi mode
   */
  ret = ioctl(adc_info.fd, SPI_IOC_WR_MODE, &adc_info.mode);
  if (ret == -1)
      pabort("can't set spi mode");

  ret = ioctl(adc_info.fd, SPI_IOC_RD_MODE, &adc_info.mode);
  if (ret == -1)
      pabort("can't get spi mode");

  /*
   * bits per word
   */
  ret = ioctl(adc_info.fd, SPI_IOC_WR_BITS_PER_WORD, &adc_info.bits_per_word);
  if (ret == -1)
      pabort("can't set bits per word");

  ret = ioctl(adc_info.fd, SPI_IOC_RD_BITS_PER_WORD, &adc_info.bits_per_word);
  if (ret == -1)
      pabort("can't get bits per word");

  /*
   * max speed hz
   */
  ret = ioctl(adc_info.fd, SPI_IOC_WR_MAX_SPEED_HZ, &adc_info.speed_hz);
  if (ret == -1)
      pabort("can't set max speed hz");

  ret = ioctl(adc_info.fd, SPI_IOC_RD_MAX_SPEED_HZ, &adc_info.speed_hz);
  if (ret == -1)
      pabort("can't get max speed hz");
  
  // configure information
  if (verbose) {
	  printf("ain: interval_ms = %d, speed = %d", interval_ms, adc_info.speed_hz);
  }

  if (log_every) {
    
    if (use_dserv) {
      // this is space for the four ADC vals
      unsigned short dserv_buf[4];
      adc_info.dserv_mqueue = mq_open(DSERV_MQUEUE_NAME, O_WRONLY);
      adc_info.adc_dpoint = dpoint_new(ain_dpoint_name, qpcs_now(),
				       DSERV_SHORT, 4,
				       (unsigned char *) dserv_buf);
    }
  }

  adc_info.interp = init_tcl();
  if (!adc_info.interp) {
    printf("error initializing Tcl interpreter\n");
    exit(0);
  }
  
  /*
   * Set up a table to hold info about specific data processors
   * available to clients to choose from using "pset"
   */
  Tcl_InitHashTable(&processFunctionTable, TCL_STRING_KEYS);
  Tcl_CreateObjCommand(adc_info.interp, "pload",
		       (Tcl_ObjCmdProc *) loadProcessCmd, 
		       (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

  Tcl_CreateObjCommand(adc_info.interp, "invert_channels",
		       (Tcl_ObjCmdProc *) invertChannelsCmd, 
		       (ClientData) &adc_info, (Tcl_CmdDeleteProc *) NULL);

  if (config_script) {
    if (Tcl_EvalFile(adc_info.interp, config_script) != TCL_OK) {
      printf("%s: %s\n", config_script, Tcl_GetStringResult(adc_info.interp));
      exit(1);
    }
  }
  
  /* call our timer setup function above */
  timer_setup(&adc_info, interval_ms);

  if (daemon) daemon_start("ain");

  for (;;) {
    sleep(100000);
  }
  
}

#if 0

/*
 * my_io_msg
 *
 *  Process QNX messages that can be more efficient than POSIX only messaging
 * 
 *   QPCS_AIN_REG_MSG: register for events to be delivered either
 *        RAW           every scan
 *        PROCESSED     after running the process code and returning true
 *
 *   This is more efficient that the notify mechanism because
 *   the pulse doesn't need to be rearmed after each delivery
 */

int my_io_msg (resmgr_context_t *ctp, io_msg_t *msg, RESMGR_OCB_T *ocb)
{
  qpcs_msg_t qpcs_msg, qpcs_reply;
  MsgRead (ctp->rcvid, &qpcs_msg, sizeof (qpcs_msg), 0);
  switch (qpcs_msg.hdr.mgrid) {
  case QPCS_AIN_REG_MSG:
    ocb->event = qpcs_msg.qpcs_reg.event;
    ocb->rcvid = ctp->rcvid;

    /* for this server, the subtype determines pulse type (raw or processed) */
    ocb->deliverEvent = qpcs_msg.hdr.subtype;
    MsgReply( ctp->rcvid, 0, &qpcs_reply, sizeof(qpcs_reply));
    return (_RESMGR_NOREPLY);
    break;
  default:
    return (ENOSYS);
    break;
  }

}



/*
 * ocb_calloc
 *
 * The purpose of this is to give us a place to allocate our own OCB. 
 * It is called as a result of the open being done
 * (e.g., iofunc_open_default causes it to be called). We
 * registered it through the mount structure. 
 */
IOFUNC_OCB_T
*ocb_calloc (resmgr_context_t *ctp, IOFUNC_ATTR_T *device)
{
  struct ocb *ocb;
  if (!(ocb = calloc (1, sizeof (*ocb)))) {
    return 0;
  }

  ocb->maxitems = device->maxitems;
  ocb->ctp = *ctp;

  ocb->process = NULL;
  ocb->process_params = NULL;
  
  ocb -> prev = &device -> list;
  if ((ocb -> next = device -> list)) {
    device -> list -> prev = &ocb -> next;
  }
  device -> list = ocb;
  return (ocb);
}


/*
 * ocb_free
 *
 * The purpose of this is
 * It is called as a result of the close being done
 * (e.g., iofunc_close_ocb_default causes it to be called). We
 * registered it through the mount structure. 
 */
void
ocb_free (IOFUNC_OCB_T *ocb)
{
	item_t *item, *nextitem;

  if ((*ocb->prev = ocb->next)) {
    ocb -> next -> prev = ocb -> prev;
  }

  /* free up any items in this OCB */
  for (nextitem = ocb->firstitem; nextitem; ) {
    if (nextitem->data) free(nextitem->data);
    item = nextitem;
    nextitem = item->next;
    free(item);
  }

  if (ocb->process_params) {
    ocb->free_params(ocb->process_params);
  }
  
  free (ocb);
}


/*
 *  io_read
 *
 *  At this point, the client has called the library read()
 *  function, and expects zero or more bytes.  Since this is
 *  the /dev/Null resource manager, we return zero bytes to
 *  indicate EOF -- no more bytes expected.
 */

/* The message that we received can be accessed via the
 * pointer *msg. A pointer to the OCB that belongs to this
 * read is the *ocb. The *ctp pointer points to a context
 * structure that is used by the resource manager framework
 * to determine whom to reply to, and more. */
int
io_read (resmgr_context_t *ctp, io_read_t *msg, RESMGR_OCB_T *ocb)
{
  device_attr_t *dattr = (device_attr_t *) ocb->hdr.attr;
  int status;

  /* Here we verify if the client has the access
   * rights needed to read from our device */
  if ((status = iofunc_read_verify(ctp, msg, &ocb->hdr, NULL)) != EOK) {
    return (status);
  }
  
  /* We check if our read callback was called because of
   * a pread() or a normal read() call. If pread(), we return
   * with an error code indicating that we don't support it.*/
  if ((msg->i.xtype & _IO_XTYPE_MASK) != _IO_XTYPE_NONE) {
    return (ENOSYS);
  }
  
  //	printf("client requested %d bytes\n", msg->i.nbytes);
  
  if (msg->i.nbytes == 4 ||
      msg->i.nbytes == 6 ||
      msg->i.nbytes == 8) {   // just read raw values
    
    /* set up the number of bytes (returned by client's read()) */
    _IO_SET_READ_NBYTES (ctp, msg->i.nbytes);
    /*
     * write the bytes to the client's reply buffer now since we
     * are about to free the data
     */
    resmgr_msgwrite (ctp, dattr->adc_info->vals, msg->i.nbytes, 0);
  }
  
  else if (ocb->firstitem) {
    size_t nbytes;
    item_t  *item, *prev;
    /* get last item */
    item = ocb->firstitem; prev = NULL;
    while (item->next != NULL) {
      prev = item;
      item = item->next;
    }
    /*
     * figure out number of bytes to give, write the data to the
     * client's reply buffer, even if we have more bytes than they
     * are asking for, we remove the item from our list
     */
    nbytes = min (strlen(item->data)+1, msg->i.nbytes);
    /* set up the number of bytes (returned by client's read()) */
    _IO_SET_READ_NBYTES (ctp, nbytes);
    /*
     * write the bytes to the client's reply buffer now since we
     * are about to free the data
     */

    resmgr_msgwrite (ctp, item->data, nbytes, 0);
    
    /* remove the data from the queue */
    if (prev)
      prev->next = item->next;
    else
      ocb->firstitem = NULL;
    free(item->data);
    free(item);
    ocb->nitems--;
    if (ocb->overflow) ocb->overflow = 0;
  }
  else {
    /* the read() will return with 0 bytes */
    _IO_SET_READ_NBYTES (ctp, 0);
  }
  
  /* mark the access time as invalid (we just accessed it) */
  if (msg->i.nbytes > 0)
    ocb->hdr.attr->attr.flags |= IOFUNC_ATTR_ATIME;
  return (EOK);
}

int io_close_dup( resmgr_context_t *ctp, io_close_t* msg,
		RESMGR_OCB_T *ocb)
{
  device_attr_t *dattr = (device_attr_t *) ocb->hdr.attr;
  /*
   * A client has closed its file descriptor or has terminated. 
   * Unblock any threads waiting for notification, then
   * remove the client from the notification list.
   */
  
  iofunc_notify_trigger_strict( ctp, dattr->notify,
				INT_MAX, IOFUNC_NOTIFY_INPUT);
  iofunc_notify_trigger_strict( ctp, dattr->notify,
				INT_MAX, IOFUNC_NOTIFY_OUTPUT);
  iofunc_notify_trigger_strict( ctp, dattr->notify,
				INT_MAX, IOFUNC_NOTIFY_OBAND );

  iofunc_notify_remove(ctp, dattr->notify);
     
  return (iofunc_close_dup_default(ctp, msg, &ocb->hdr));
}

/*
 *  io_write
 *
 *  At this point, the client has called the library write()
 *  function, and expects that our resource manager will write
 *  the number of bytes that have been specified to the device.
 *
 *  Since this is /dev/Null, all of the clients' writes always
 *  work -- they just go into Deep Outer Space.
 */

int
io_write (resmgr_context_t *ctp, io_write_t *msg, RESMGR_OCB_T *ocb)
{
  int status;
  char *buf;
  static int allocated_buf = 0;
  device_attr_t *dattr = (device_attr_t *) ocb->hdr.attr;
  
  /* Check the access permissions of the client */
  if ((status = iofunc_write_verify(ctp, msg, &ocb->hdr, NULL)) != EOK) {
    return (status);
  }
  
  /* Check if pwrite() or normal write() */
  if ((msg->i.xtype & _IO_XTYPE_MASK) != _IO_XTYPE_NONE) {
    return (ENOSYS);
  }
  
  /* Set the number of bytes successfully written for
   * the client. This information will be passed to the
   * client by the resource manager framework upon reply.
   * In this example, we just take the number of  bytes that
   * were sent to us and claim we successfully wrote them. */
  _IO_SET_WRITE_NBYTES (ctp, msg -> i.nbytes);
  
  /* Here we print the data. This is a good example for the case
   * where you actually would like to do something with the data.
   */
  /* First check if our message buffer was large enough
   * to receive the whole write at once. If yes, print data.*/
  if( (msg->i.nbytes <= ctp->info.msglen - ctp->offset - sizeof(msg->i)) &&
      (ctp->info.msglen < ctp->msg_max_size))  { // space for NUL byte
    allocated_buf = 0;
    buf = (char *)(msg+1);
    buf[msg->i.nbytes] = '\0';
  } else {
    /* If we did not receive the whole message because the
     * client wanted to send more than we could receive, we
     * allocate memory for all the data and use resmgr_msgread()
     * to read all the data at once. Although we did not receive
     * the data completely first, because our buffer was not big
     * enough, the data is still fully available on the client
     * side, because its write() call blocks until we return
     * from this callback! */
    buf = malloc( msg->i.nbytes + 1);
    allocated_buf = 1;
    resmgr_msgread( ctp, buf, msg->i.nbytes, sizeof(msg->i));
    buf[msg->i.nbytes] = '\0';
  }
  
  /* Finally, if we received more than 0 bytes, we mark the
   * file information for the device to be updated:
   * modification time and change of file status time. To
   * avoid constant update of the real file status information
   * (which would involve overhead getting the current time), we
   * just set these flags. The actual update is done upon
   * closing, which is valid according to POSIX. */
  if (msg->i.nbytes > 0) {
    int buflen = strlen(buf);
    if (buf[buflen-1] == '\n')
      buf[buflen-1] = '\0';
    int argc;
    char **argv;
    int ndx;
    if (Tcl_SplitList(dattr->interp, buf, &argc,
		      (const char ***) &argv) == TCL_OK) {
      if (argc) {
    	if (!strcmp(argv[0], "pset") && argc == 2) {
	  setProcess(dattr->interp,  argv[1], ocb);
	}
	else if (!strcmp(argv[0], "pparam")) {
	  if (argc == 3) {
	    setProcessParam(dattr, argv[1], argv[2], -1, ocb);
	  }
	  else if (argc == 4) {
	    ndx = atoi(argv[3]);
	    setProcessParam(dattr, argv[1], argv[2], ndx, ocb);
	  }
	}
	else if (!strcmp(argv[0], "gparam")) {
	  if (argc == 2) {
	    if (getProcessParam(dattr, argv[1], -1, ocb) == TCL_OK) {
	      add_to_queue(dattr, ocb, Tcl_GetStringResult(dattr->interp));
	    }
	  }
	  else if (argc == 3) {
	    ndx = atoi(argv[2]);
	    if (getProcessParam(dattr, argv[1], ndx, ocb) == TCL_OK) {
	      add_to_queue(dattr, ocb, Tcl_GetStringResult(dattr->interp));
	    }
	  }
	}
	else {
	  Tcl_Eval(dattr->interp, buf);
	}
      }
      ckfree(argv);
    }
    
    if (allocated_buf) free(buf);
    ocb->hdr.attr->attr.flags |= IOFUNC_ATTR_MTIME | IOFUNC_ATTR_CTIME;
  }
  
  return (_RESMGR_NPARTS (0));
}

int
io_notify( resmgr_context_t *ctp, io_notify_t *msg,
	   RESMGR_OCB_T *ocb)
{
  device_attr_t *dattr = (device_attr_t *) ocb->hdr.attr;
  int trig;
  /*
   * 'trig' will tell iofunc_notify() which conditions are
   * currently satisfied. 'dattr->nitems' is the number of
   * messages in our list of stored messages.
   */
  trig = _NOTIFY_COND_OUTPUT; /* clients can always give us data */
  if (ocb->nitems > 0)
    trig |= _NOTIFY_COND_INPUT; /* we have some data available */
  /*
   * iofunc_notify() will do any necessary handling, including 
   * adding the client to the notification list if need be.
   */
  ocb->notify = 1;
  return (iofunc_notify( ctp, msg, dattr->notify, trig, NULL, NULL));
}

#endif
