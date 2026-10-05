#ifndef __STORVSC_H
#define __STORVSC_H

struct disk_op_s;
int storvsc_process_op(struct disk_op_s *op);
void storvsc_setup(void);

#endif // storvsc.h
