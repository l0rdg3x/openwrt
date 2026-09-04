// SPDX-License-Identifier: GPL-2.0-only

#include <linux/bug.h>
#include <linux/errno.h>
#include <linux/iopoll.h>
#include <linux/mutex.h>
#include <asm/mach-rtl-otto/mach-rtl-otto.h>

#include "table.h"

/* One table access register: a command register selecting table and index, and
 * a window of data registers holding the entry. tbl and width belong to the
 * table currently held, not to the register.
 */
struct otto_table {
	u16 addr;
	u16 data;
	u8  max_data;
	u8 c_bit;
	u8 t_bit;
	u8 rmode;
	struct mutex lock;
};

#define TBL_DESC(_addr, _data, _max_data, _c_bit, _t_bit, _rmode) \
		{  .addr = _addr, .data = _data, .max_data = _max_data, .c_bit = _c_bit, \
		    .t_bit = _t_bit, .rmode = _rmode \
		}

enum otto_table_reg {
	OTTO_REG_8380_L2 = 0,
	OTTO_REG_8380_0,
	OTTO_REG_8380_1,
	OTTO_REG_8390_L2,
	OTTO_REG_8390_0,
	OTTO_REG_8390_1,
	OTTO_REG_8390_2,
	OTTO_REG_9300_L2,
	OTTO_REG_9300_0,
	OTTO_REG_9300_1,
	OTTO_REG_9300_2,
	OTTO_REG_9300_HSB,
	OTTO_REG_9300_HSA,
	OTTO_REG_9310_0,
	OTTO_REG_9310_1,
	OTTO_REG_9310_2,
	OTTO_REG_9310_3,
	OTTO_REG_9310_4,
	OTTO_REG_9310_5,
	OTTO_REG_END
};

static struct otto_table otto_regs[] = {
	TBL_DESC(0x6900, 0x6908, 3, 15, 13, 1),		/* OTTO_REG_8380_L2 */
	TBL_DESC(0x6914, 0x6918, 18, 14, 12, 1),	/* OTTO_REG_8380_0 */
	TBL_DESC(0xA4C8, 0xA4CC, 6, 14, 12, 1),		/* OTTO_REG_8380_1 */

	TBL_DESC(0x1180, 0x1184, 3, 16, 14, 0),		/* OTTO_REG_8390_L2 */
	TBL_DESC(0x1190, 0x1194, 17, 15, 12, 0),	/* OTTO_REG_8390_0 */
	TBL_DESC(0x6B80, 0x6B84, 4, 14, 12, 0),		/* OTTO_REG_8390_1 */
	TBL_DESC(0x611C, 0x6120, 9, 8, 6, 0),		/* OTTO_REG_8390_2 */

	TBL_DESC(0xB320, 0xB334, 3, 18, 16, 0),		/* OTTO_REG_9300_L2 */
	TBL_DESC(0xB340, 0xB344, 19, 16, 12, 0),	/* OTTO_REG_9300_0 */
	TBL_DESC(0xB3A0, 0xB3A4, 20, 16, 13, 0),	/* OTTO_REG_9300_1 */
	TBL_DESC(0xCE04, 0xCE08, 6, 14, 12, 0),		/* OTTO_REG_9300_2 */
	TBL_DESC(0xD600, 0xD604, 30, 7, 6, 0),		/* OTTO_REG_9300_HSB */
	TBL_DESC(0x7880, 0x7884, 22, 9, 8, 0),		/* OTTO_REG_9300_HSA */

	TBL_DESC(0x8500, 0x8508, 8, 19, 15, 0),		/* OTTO_REG_9310_0 */
	TBL_DESC(0x40C0, 0x40C4, 22, 16, 14, 0),	/* OTTO_REG_9310_1 */
	TBL_DESC(0x8528, 0x852C, 6, 18, 14, 0),		/* OTTO_REG_9310_2 */
	TBL_DESC(0x0200, 0x0204, 9, 15, 12, 0),		/* OTTO_REG_9310_3 */
	TBL_DESC(0x20dc, 0x20e0, 29, 7, 6, 0),		/* OTTO_REG_9310_4 */
	TBL_DESC(0x7e1c, 0x7e20, 53, 8, 6, 0),		/* OTTO_REG_9310_5 */
};

/* Which access register selects a table, the type value it is selected with,
 * and how many data registers one of its entries occupies.
 */
struct otto_table_map {
	u8 reg;
	u8 type;
	u8 width;
};

#define TBL_MAP(_id, _reg, _type, _width)	\
	[OTTO_TBL_HANDLE(_id)] = { .reg = _reg, .type = _type, .width = _width }

/* A transfer takes its length from here, so no entry may be wider than the
 * data window of the register it sits on.
 */
static const struct otto_table_map otto_table_maps[OTTO_TBL_COUNT] = {
	TBL_MAP(RTL8380_TBL_L2_UC, OTTO_REG_8380_L2, 0, 3),
	TBL_MAP(RTL8380_TBL_L2_IP_MC, OTTO_REG_8380_L2, 0, 3),
	TBL_MAP(RTL8380_TBL_L2_IP_MC_SIP, OTTO_REG_8380_L2, 0, 3),
	TBL_MAP(RTL8380_TBL_L2_MC, OTTO_REG_8380_L2, 0, 3),
	TBL_MAP(RTL8380_TBL_L2_NEXT_HOP, OTTO_REG_8380_L2, 0, 3),
	TBL_MAP(RTL8380_TBL_L2_NEXT_HOP_LEGACY, OTTO_REG_8380_L2, 0, 3),
	TBL_MAP(RTL8380_TBL_L2_CAM_UC, OTTO_REG_8380_L2, 1, 3),
	TBL_MAP(RTL8380_TBL_L2_CAM_IP_MC, OTTO_REG_8380_L2, 1, 3),
	TBL_MAP(RTL8380_TBL_L2_CAM_IP_MC_SIP, OTTO_REG_8380_L2, 1, 3),
	TBL_MAP(RTL8380_TBL_L2_CAM_MC, OTTO_REG_8380_L2, 1, 3),
	TBL_MAP(RTL8380_TBL_MC_PMSK, OTTO_REG_8380_L2, 2, 1),

	TBL_MAP(RTL8380_TBL_VLAN, OTTO_REG_8380_0, 0, 2),
	TBL_MAP(RTL8380_TBL_IACL, OTTO_REG_8380_0, 1, 18),
	TBL_MAP(RTL8380_TBL_MSTI, OTTO_REG_8380_0, 2, 2),
	TBL_MAP(RTL8380_TBL_LOG, OTTO_REG_8380_0, 3, 2),

	TBL_MAP(RTL8380_TBL_UNTAG, OTTO_REG_8380_1, 0, 1),
	TBL_MAP(RTL8380_TBL_ROUTING, OTTO_REG_8380_1, 2, 2),

	TBL_MAP(RTL8390_TBL_L2_UC, OTTO_REG_8390_L2, 0, 3),
	TBL_MAP(RTL8390_TBL_L2_IP_MC, OTTO_REG_8390_L2, 0, 3),
	TBL_MAP(RTL8390_TBL_L2_IP_MC_SIP, OTTO_REG_8390_L2, 0, 3),
	TBL_MAP(RTL8390_TBL_L2_MC, OTTO_REG_8390_L2, 0, 3),
	TBL_MAP(RTL8390_TBL_L2_NEXT_HOP, OTTO_REG_8390_L2, 0, 3),
	TBL_MAP(RTL8390_TBL_L2_NH_LEGACY, OTTO_REG_8390_L2, 0, 3),
	TBL_MAP(RTL8390_TBL_L2_CAM_UC, OTTO_REG_8390_L2, 1, 3),
	TBL_MAP(RTL8390_TBL_L2_CAM_IP_MC, OTTO_REG_8390_L2, 1, 3),
	TBL_MAP(RTL8390_TBL_L2_CAM_IP_MC_SIP, OTTO_REG_8390_L2, 1, 3),
	TBL_MAP(RTL8390_TBL_L2_CAM_MC, OTTO_REG_8390_L2, 1, 3),
	TBL_MAP(RTL8390_TBL_MC_PMSK, OTTO_REG_8390_L2, 2, 2),

	TBL_MAP(RTL8390_TBL_VLAN, OTTO_REG_8390_0, 0, 3),
	TBL_MAP(RTL8390_TBL_IACL, OTTO_REG_8390_0, 2, 17),
	TBL_MAP(RTL8390_TBL_EACL, OTTO_REG_8390_0, 2, 17),
	TBL_MAP(RTL8390_TBL_LOG, OTTO_REG_8390_0, 4, 2),
	TBL_MAP(RTL8390_TBL_MSTI, OTTO_REG_8390_0, 5, 4),

	TBL_MAP(RTL8390_TBL_UNTAG, OTTO_REG_8390_1, 0, 2),
	TBL_MAP(RTL8390_TBL_ROUTING, OTTO_REG_8390_1, 2, 2),

	TBL_MAP(RTL9300_TBL_L2_UC, OTTO_REG_9300_L2, 0, 3),
	TBL_MAP(RTL9300_TBL_L2_MC, OTTO_REG_9300_L2, 0, 3),
	TBL_MAP(RTL9300_TBL_L2_CAM_UC, OTTO_REG_9300_L2, 1, 3),
	TBL_MAP(RTL9300_TBL_L2_CAM_MC, OTTO_REG_9300_L2, 1, 3),
	TBL_MAP(RTL9300_TBL_MC_PORTMASK, OTTO_REG_9300_L2, 2, 1),

	TBL_MAP(RTL9300_TBL_VLAN, OTTO_REG_9300_0, 1, 2),
	TBL_MAP(RTL9300_TBL_IACL, OTTO_REG_9300_0, 2, 19),
	TBL_MAP(RTL9300_TBL_VACL, OTTO_REG_9300_0, 2, 19),
	TBL_MAP(RTL9300_TBL_LOG, OTTO_REG_9300_0, 3, 2),
	TBL_MAP(RTL9300_TBL_MSTI, OTTO_REG_9300_0, 4, 2),
	TBL_MAP(RTL9300_TBL_PORT_ISO_CTRL, OTTO_REG_9300_0, 6, 1),
	TBL_MAP(RTL9300_TBL_LAG, OTTO_REG_9300_0, 7, 3),
	TBL_MAP(RTL9300_TBL_SRC_TRK_MAP, OTTO_REG_9300_0, 8, 1),

	TBL_MAP(RTL9300_TBL_L3_ROUTER_MAC, OTTO_REG_9300_1, 0, 7),
	TBL_MAP(RTL9300_TBL_L3_HOST_ROUTE_IPUC, OTTO_REG_9300_1, 1, 5),
	TBL_MAP(RTL9300_TBL_L3_HOST_ROUTE_IP6MC, OTTO_REG_9300_1, 1, 11),
	TBL_MAP(RTL9300_TBL_L3_HOST_ROUTE_IP6UC, OTTO_REG_9300_1, 1, 5),
	TBL_MAP(RTL9300_TBL_L3_HOST_ROUTE_IPMC, OTTO_REG_9300_1, 1, 11),
	TBL_MAP(RTL9300_TBL_L3_PREFIX_ROUTE_IPUC, OTTO_REG_9300_1, 2, 11),
	TBL_MAP(RTL9300_TBL_L3_PREFIX_ROUTE_IP6MC, OTTO_REG_9300_1, 2, 20),
	TBL_MAP(RTL9300_TBL_L3_PREFIX_ROUTE_IP6UC, OTTO_REG_9300_1, 2, 11),
	TBL_MAP(RTL9300_TBL_L3_PREFIX_ROUTE_IPMC, OTTO_REG_9300_1, 2, 20),
	TBL_MAP(RTL9300_TBL_L3_NEXTHOP, OTTO_REG_9300_1, 3, 1),
	TBL_MAP(RTL9300_TBL_L3_EGR_INTF, OTTO_REG_9300_1, 4, 2),

	TBL_MAP(RTL9300_TBL_UNTAG, OTTO_REG_9300_2, 0, 1),
	TBL_MAP(RTL9300_TBL_L3_EGR_INTF_MAC, OTTO_REG_9300_2, 2, 2),

	TBL_MAP(RTL9310_TBL_L2_UC, OTTO_REG_9310_0, 0, 4),
	TBL_MAP(RTL9310_TBL_FT_L2_HASH_FMT0_0, OTTO_REG_9310_0, 0, 4),
	TBL_MAP(RTL9310_TBL_FT_L2_HASH_FMT0_1, OTTO_REG_9310_0, 0, 4),
	TBL_MAP(RTL9310_TBL_FT_L2_HASH_FMT1_0, OTTO_REG_9310_0, 0, 4),
	TBL_MAP(RTL9310_TBL_FT_L2_HASH_FMT1_1, OTTO_REG_9310_0, 0, 4),
	TBL_MAP(RTL9310_TBL_FT_L2_HASH_FMT2_0, OTTO_REG_9310_0, 0, 4),
	TBL_MAP(RTL9310_TBL_FT_L2_HASH_FMT2_1, OTTO_REG_9310_0, 0, 4),
	TBL_MAP(RTL9310_TBL_L2_CB_MC, OTTO_REG_9310_0, 0, 3),
	TBL_MAP(RTL9310_TBL_L2_CB_UC, OTTO_REG_9310_0, 0, 4),
	TBL_MAP(RTL9310_TBL_L2_MC, OTTO_REG_9310_0, 0, 4),
	TBL_MAP(RTL9310_TBL_L2_TNL_MCAST, OTTO_REG_9310_0, 0, 3),
	TBL_MAP(RTL9310_TBL_L2_TNL_UCAST, OTTO_REG_9310_0, 0, 3),
	TBL_MAP(RTL9310_TBL_PE_FWD, OTTO_REG_9310_0, 0, 4),
	TBL_MAP(RTL9310_TBL_WLC_MCAST, OTTO_REG_9310_0, 0, 3),
	TBL_MAP(RTL9310_TBL_WLC_UCAST, OTTO_REG_9310_0, 0, 3),
	TBL_MAP(RTL9310_TBL_L2_CAM_UC, OTTO_REG_9310_0, 1, 4),
	TBL_MAP(RTL9310_TBL_FT_L2_CAM_FMT0_0, OTTO_REG_9310_0, 1, 4),
	TBL_MAP(RTL9310_TBL_FT_L2_CAM_FMT0_1, OTTO_REG_9310_0, 1, 4),
	TBL_MAP(RTL9310_TBL_FT_L2_CAM_FMT1_0, OTTO_REG_9310_0, 1, 4),
	TBL_MAP(RTL9310_TBL_FT_L2_CAM_FMT1_1, OTTO_REG_9310_0, 1, 4),
	TBL_MAP(RTL9310_TBL_FT_L2_CAM_FMT2_0, OTTO_REG_9310_0, 1, 4),
	TBL_MAP(RTL9310_TBL_L2_CAM_CB_MC, OTTO_REG_9310_0, 1, 3),
	TBL_MAP(RTL9310_TBL_L2_CAM_CB_UC, OTTO_REG_9310_0, 1, 4),
	TBL_MAP(RTL9310_TBL_L2_CAM_MC, OTTO_REG_9310_0, 1, 4),
	TBL_MAP(RTL9310_TBL_L2_TNL_MCAST_CAM, OTTO_REG_9310_0, 1, 3),
	TBL_MAP(RTL9310_TBL_L2_TNL_UCAST_CAM, OTTO_REG_9310_0, 1, 3),
	TBL_MAP(RTL9310_TBL_PE_FWD_CAM, OTTO_REG_9310_0, 1, 4),
	TBL_MAP(RTL9310_TBL_WLC_MCAST_CAM, OTTO_REG_9310_0, 1, 3),
	TBL_MAP(RTL9310_TBL_WLC_UCAST_CAM, OTTO_REG_9310_0, 1, 3),
	TBL_MAP(RTL9310_TBL_MC_PMSK, OTTO_REG_9310_0, 2, 2),
	TBL_MAP(RTL9310_TBL_VLAN, OTTO_REG_9310_0, 3, 4),
	TBL_MAP(RTL9310_TBL_MSTI, OTTO_REG_9310_0, 5, 4),
	TBL_MAP(RTL9310_TBL_SRC_TRK_MAP, OTTO_REG_9310_0, 13, 1),

	TBL_MAP(RTL9310_TBL_IACL, OTTO_REG_9310_1, 0, 22),
	TBL_MAP(RTL9310_TBL_EACL, OTTO_REG_9310_1, 0, 22),
	TBL_MAP(RTL9310_TBL_FT_EGR, OTTO_REG_9310_1, 0, 17),
	TBL_MAP(RTL9310_TBL_FT_IGR, OTTO_REG_9310_1, 0, 22),
	TBL_MAP(RTL9310_TBL_VACL, OTTO_REG_9310_1, 0, 22),

	TBL_MAP(RTL9310_TBL_LAG, OTTO_REG_9310_2, 0, 3),
	TBL_MAP(RTL9310_TBL_PORT_ISO_CTRL, OTTO_REG_9310_2, 1, 2),

	TBL_MAP(RTL9310_TBL_VLAN_UNTAG, OTTO_REG_9310_3, 0, 2),

	TBL_MAP(RTL9310_TBL_STAT_PORT_MIB_CNTR, OTTO_REG_9310_5, 0, 53),
	TBL_MAP(RTL9310_TBL_STAT_PORT_PRVTE_CNTR, OTTO_REG_9310_5, 1, 28),
};

void otto_table_init(void)
{
	for (int i = 0; i < OTTO_REG_END; i++)
		mutex_init(&otto_regs[i].lock);
}

static int otto_table_id_to_handle(enum otto_table_id id)
{
	if (id < OTTO_TBL_ID_BASE || id >= OTTO_TBL_END)
		return -EINVAL;

	return OTTO_TBL_HANDLE(id);
}

static enum otto_table_id otto_table_handle_to_id(int handle)
{
	return handle + OTTO_TBL_ID_BASE;
}

/* The register a table is reached through. NULL for a handle out of range,
 * which every entry point checks before touching the hardware.
 */
static struct otto_table *otto_table_reg(int handle)
{
	if (handle < 0 || handle >= OTTO_TBL_COUNT)
		return NULL;

	return &otto_regs[otto_table_maps[handle].reg];
}

/* Take a table and hold it until otto_table_release() */
int otto_table_acquire(enum otto_table_id id)
{
	int handle = otto_table_id_to_handle(id);
	struct otto_table *r = otto_table_reg(handle);

	if (!r)
		return -EINVAL;

	mutex_lock(&r->lock);

	return handle;
}

void otto_table_release(int handle)
{
	struct otto_table *r = otto_table_reg(handle);

	if (!r)
		return;

	mutex_unlock(&r->lock);
}

static int otto_table_exec(struct otto_table *r, u8 type, bool is_write, int idx)
{
	int ret = 0;
	u32 cmd, val;

	/* Read/write bit has inverted meaning on RTL838x */
	if (r->rmode)
		cmd = is_write ? 0 : BIT(r->c_bit);
	else
		cmd = is_write ? BIT(r->c_bit) : 0;

	cmd |= BIT(r->c_bit + 1); /* Execute bit */
	cmd |= type << r->t_bit; /* Table type */
	cmd |= idx & (BIT(r->t_bit) - 1); /* Index */

	sw_w32(cmd, r->addr);

	ret = readx_poll_timeout(sw_r32, r->addr, val,
				 !(val & BIT(r->c_bit + 1)), 20, 10000);
	if (ret)
		pr_err("%s: timeout\n", __func__);

	return ret;
}

static u16 otto_table_word_addr(struct otto_table *r, int word)
{
	if (word >= r->max_data)
		word = r->max_data - 1;

	return r->data + word * 4;
}

int __otto_table_fetch(int handle, int idx)
{
	struct otto_table *r = otto_table_reg(handle);

	if (!r)
		return -EINVAL;

	return otto_table_exec(r, otto_table_maps[handle].type, false, idx);
}

u32 __otto_table_word_read(int handle, int word)
{
	struct otto_table *r = otto_table_reg(handle);

	if (!r)
		return 0;

	return sw_r32(otto_table_word_addr(r, word));
}

int __otto_table_read_bytes(int handle, int idx, void *buf, size_t size)
{
	struct otto_table *r = otto_table_reg(handle);
	unsigned int words;
	u32 *out = buf;
	int ret;

	if (!r)
		return -EINVAL;

	words = otto_table_maps[handle].width;
	if (WARN_ONCE(size != words * sizeof(u32),
		      "otto_table: %zu bytes for table %d, a %u word entry\n",
		      size, otto_table_handle_to_id(handle), words))
		return -EINVAL;

	ret = otto_table_exec(r, otto_table_maps[handle].type, false, idx);
	if (ret)
		return ret;

	for (unsigned int i = 0; i < words; i++)
		out[i] = sw_r32(r->data + i * 4);

	return 0;
}

int __otto_table_write_bytes(int handle, int idx, const void *buf, size_t size)
{
	struct otto_table *r = otto_table_reg(handle);
	unsigned int words;
	const u32 *in = buf;

	if (!r)
		return -EINVAL;

	words = otto_table_maps[handle].width;
	if (WARN_ONCE(size != words * sizeof(u32),
		      "otto_table: %zu bytes for table %d, a %u word entry\n",
		      size, otto_table_handle_to_id(handle), words))
		return -EINVAL;

	for (unsigned int i = 0; i < words; i++)
		sw_w32(in[i], r->data + i * 4);

	return otto_table_exec(r, otto_table_maps[handle].type, true, idx);
}

int otto_table_read_bytes(void *buf, size_t size, enum otto_table_id id, int idx)
{
	int handle = otto_table_acquire(id);
	int ret;

	if (handle < 0)
		return handle;

	ret = __otto_table_read_bytes(handle, idx, buf, size);
	otto_table_release(handle);

	return ret;
}

int otto_table_write_bytes(enum otto_table_id id, int idx, const void *buf, size_t size)
{
	int handle = otto_table_acquire(id);
	int ret;

	if (handle < 0)
		return handle;

	ret = __otto_table_write_bytes(handle, idx, buf, size);
	otto_table_release(handle);

	return ret;
}
