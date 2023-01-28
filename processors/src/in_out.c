#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include <math.h>

#include <datapoint.h>
#include <ain_process.h>
#include <prmutil.h>

typedef struct process_params_s {
  int state;			/* are we > threshold */
  float threshold;		/* what is threshold? */
  float scale;			/* scale factor       */
  ds_datapoint_t *dpoint;
} process_params_t;


void *newProcessParams(void)
{
  process_params_t *p = calloc(1, sizeof(process_params_t));
  p->state = -1;
  p->threshold = 9.0;
  p->scale = 0.005;

  p->dpoint = calloc(1, sizeof(ds_datapoint_t));
  p->dpoint->flags = 0;
  p->dpoint->varname = strdup("ain/proc/in_out");
  p->dpoint->varlen = strlen(p->dpoint->varname)+1;
  p->dpoint->data.type = DSERV_SHORT;
  p->dpoint->data.len = 3*sizeof(uint16_t);
  p->dpoint->data.buf = malloc(p->dpoint->data.len);

  return p;
}

int freeProcessParams(void *pstruct)
{
  process_params_t *p = (process_params_t *) pstruct;
  free(p->dpoint->varname);
  free(p->dpoint->data.buf);
  free(p->dpoint);
  free(p);
  return 0;
}


int getProcessParams(ain_param_setting_t *pinfo)
{
  char *result_str;
  char *name = pinfo->pname;
  process_params_t *p = (process_params_t *) pinfo->params;
  
  int dummyInt;

  PARAM_ENTRY params[] = {
			  { "state",      &p->state,            &dummyInt,   PU_INT },
			  { "threshold",  &p->threshold,        &dummyInt,   PU_FLOAT },
			  { "scale",      &p->scale,            &dummyInt,   PU_FLOAT },
			  { "", NULL, NULL, PU_NULL }
  };

  result_str = puGetParamEntry(&params[0], name);
  if (result_str && pinfo->pval) {
    *pinfo->pval = result_str;
    return 1;
  }		 
  return 0;
}

int setProcessParams(ain_param_setting_t *pinfo)
{
  char *name = pinfo->pname;
  char **vals = pinfo->pval;
  process_params_t *p = (process_params_t *) pinfo->params;

  int dummyInt;
  PARAM_ENTRY params[] = {
			  { "state",      &p->state,            &dummyInt,   PU_INT } ,
			  { "threshold",  &p->threshold,        &dummyInt,   PU_FLOAT },
			  { "scale",      &p->scale,            &dummyInt,   PU_FLOAT },
			  { "", NULL, NULL, PU_NULL }
  };

  /* if the special dpoint param name is passed, changed the dpoint name */
  if (!strcmp(name, "dpoint")) {
    if (p->dpoint->varname) free(p->dpoint->varname);
    p->dpoint->varname = strdup(vals[0]);
    p->dpoint->varlen = strlen(p->dpoint->varname)+1;
  }
  else {
    puSetParamEntry(&params[0], name, 1, vals);
  }
  
  return AIN_PROCESS_IGNORE;
}

int onProcess(process_info_t *pinfo, void *params)
{
  process_params_t *p = (process_params_t *) params;
  float scale = p->scale;
  float threshold = p->threshold;
  float x = pinfo->v[1];
  float y = pinfo->v[0];
  x = (x-2048.)*scale;
  y = (y-2048.)*scale;
  int retval = AIN_PROCESS_IGNORE;
  
  if (x*x+y*y > threshold) {
    if (p->state < 0 || !p->state) {
      p->state = 1;
      retval = AIN_PROCESS_DSERV;
    }
  }
  else {
    if (p->state) {
      p->state = 0;
      retval = AIN_PROCESS_DSERV;
    }
  }

  if (retval == AIN_PROCESS_DSERV) {
    uint16_t *vals = (uint16_t *) p->dpoint->data.buf;
    vals[0] = p->state;
    vals[1] = pinfo->v[1];
    vals[2] = pinfo->v[0];
    p->dpoint->timestamp = pinfo->timestamp;
    pinfo->dpoint = p->dpoint;
  }

  return retval;
}
