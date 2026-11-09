#ifndef ICV_H
#define ICV_H

#ifdef __cplusplus
extern "C" {
#endif

void icv_init(void);   /* call once after all MX_..._Init() */
void icv_loop(void);   /* call from while(1)               */

#ifdef __cplusplus
}
#endif

#endif /* ICV_H */
