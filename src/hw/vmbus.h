#ifndef __VMBUS_H
#define __VMBUS_H

#include "types.h" // u32

// VMBus packet types (vmpacket_descriptor.type)
#define VM_PKT_DATA_INBAND              6
#define VM_PKT_DATA_USING_GPA_DIRECT    9
#define VM_PKT_COMP                     11

struct vmbus_gpa_range {
    u32 byte_count;
    u32 byte_offset;
    u64 pfn_array[];
} PACKED;

struct vmbus_channel {
    u32 child_relid;
    u32 connection_id;
    struct vmbus_ring *out, *in;
    u32 out_size, in_size;      // data area sizes in bytes
};

int vmbus_setup(void);
int vmbus_find_offers(const u8 *if_type, u32 *relids, u32 *connids, int max);
int vmbus_open(struct vmbus_channel *chan, u32 relid, u32 connid
               , u32 ring_pages);
int vmbus_send(struct vmbus_channel *chan, u16 type
               , struct vmbus_gpa_range *range, u32 range_len
               , void *data, u32 len, u64 trans_id);
int vmbus_recv(struct vmbus_channel *chan, void *data, u32 maxlen
               , u16 *type, u64 *trans_id);

#endif // vmbus.h
