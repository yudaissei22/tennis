#include <stdio.h>
#include <stdlib.h>

#include "online_trajectory_modification_shm.h"
#include "shm_common.h"

static struct otm_shm *s_shm;

int initialize_sharedmemory() {
  s_shm = (struct otm_shm *)set_shared_memory(7777, sizeof(struct otm_shm));
  return 0;
}

int write_ref_angle_shm (int fv_size, int seq, float *fv) {
  int i, j;
  int cnt = 0;
  int n = fv_size / seq;

  if(n > MAX_JOINT_NUM) {
    fprintf(stderr, "MAX_JOINT_NUM over!\n");
    return -1;
  } else {
    for(j = 0; j < seq; j++) {
      for(i = 0; i < n; i++) {
        s_shm->ref_angle_buf[j][i] = fv[cnt++];
      }
    }
  }
  return n*seq;
}

int write_otm_flag_shm (int flag) {
  fprintf(stderr, "otm_flag = %d!\n", flag);
  s_shm->otm_flag = flag;
  return flag;
}

short read_otm_flag_shm () {
  return s_shm->otm_flag;
}
