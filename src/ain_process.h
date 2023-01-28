#ifndef AIN_PROCESS_H_
#define AIN_PROCESS_H_

enum { AIN_PROCESS_IGNORE, AIN_PROCESS_NOTIFY, AIN_PROCESS_DSERV };

typedef struct process_info_s {
  int nchannels;
  unsigned short *v;
  char **result_str;
  uint64_t timestamp;
  ds_datapoint_t *dpoint;
} process_info_t;

typedef struct ain_param_setting_s {
  char **pval;
  int index;
  char *pname;
  void *params;
  uint64_t timestamp;
  ds_datapoint_t *dpoint;
} ain_param_setting_t;
  
typedef int (*PROCESS_FUNC)(process_info_t *info, void *);
typedef void * (*PROCESS_NEWPARAM_FUNC)(void);
typedef void * (*PROCESS_FREEPARAM_FUNC)(void *);
typedef int (*PROCESS_SETPARAM_FUNC)(ain_param_setting_t *p);

#endif /* AIN_PROCESS_H_ */
