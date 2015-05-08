/** @file mlan_sta_rx.c
 *
 *  @brief This file contains the handling of RX in MLAN
 *  module.
 *
 *  Copyright (C) 2008-2014, Marvell International Ltd.
 *
 *  This software file (the "File") is distributed by Marvell International
 *  Ltd. under the terms of the GNU General Public License Version 2, June 1991
 *  (the "License").  You may use, redistribute and/or modify this File in
 *  accordance with the terms and conditions of the License, a copy of which
 *  is available by writing to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or on the
 *  worldwide web at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt.
 *
 *  THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE
 *  ARE EXPRESSLY DISCLAIMED.  The License provides additional details about
 *  this warranty disclaimer.
 */

/********************************************************
Change log:
    10/27/2008: initial version
********************************************************/

#include "mlan.h"
#include "mlan_join.h"
#include "mlan_util.h"
#include "mlan_fw.h"
#include "mlan_main.h"
#include "mlan_11n_aggr.h"
#include "mlan_11n_rxreorder.h"

/********************************************************
		Local Variables
********************************************************/

/** Ethernet II header */
typedef struct {
    /** Ethernet II header destination address */
	t_u8 dest_addr[MLAN_MAC_ADDR_LENGTH];
    /** Ethernet II header source address */
	t_u8 src_addr[MLAN_MAC_ADDR_LENGTH];
    /** Ethernet II header length */
	t_u16 ethertype;

} EthII_Hdr_t;

/** IPv4 ARP request header */
typedef MLAN_PACK_START struct {
    /** Hardware type */
	t_u16 Htype;
    /** Protocol type */
	t_u16 Ptype;
    /** Hardware address length */
	t_u8 addr_len;
    /** Protocol address length */
	t_u8 proto_len;
    /** Operation code */
	t_u16 op_code;
    /** Source mac address */
	t_u8 src_mac[MLAN_MAC_ADDR_LENGTH];
    /** Sender IP address */
	t_u8 src_ip[4];
    /** Destination mac address */
	t_u8 dst_mac[MLAN_MAC_ADDR_LENGTH];
    /** Destination IP address */
	t_u8 dst_ip[4];
} MLAN_PACK_END IPv4_ARP_t;

/** IPv6 Nadv packet header */
typedef MLAN_PACK_START struct {
    /** IP protocol version */
	t_u8 version;
    /** flow label */
	t_u8 flow_lab[3];
    /** Payload length */
	t_u16 payload_len;
    /** Next header type */
	t_u8 next_hdr;
    /** Hot limit */
	t_u8 hop_limit;
    /** Source address */
	t_u8 src_addr[16];
    /** Destination address */
	t_u8 dst_addr[16];
    /** ICMP type */
	t_u8 icmp_type;
    /** IPv6 Code */
	t_u8 ipv6_code;
    /** IPv6 Checksum */
	t_u16 ipv6_checksum;
    /** Flags */
	t_u32 flags;
    /** Target address */
	t_u8 taget_addr[16];
    /** Reserved */
	t_u8 rev[8];
} MLAN_PACK_END IPv6_Nadv_t;

/********************************************************
		Global Variables
********************************************************/

/********************************************************
		Local Functions
********************************************************/

/********************************************************
		Global functions
********************************************************/
/**
 *  @brief This function check and discard IPv4 and IPv6 gratuitous broadcast packets
 *
 *  @param prx_pkt     A pointer to RxPacketHdr_t structure of received packet
 *  @param pmadapter   A pointer to pmlan_adapter structure
 *  @return            TRUE if found such type of packets, FALSE not found
 */
static t_u8
discard_gratuitous_ARP_msg(RxPacketHdr_t * prx_pkt, pmlan_adapter pmadapter)
{
	t_u8 proto_ARP_type[] = { 0x08, 0x06 };
	t_u8 proto_ARP_type_v6[] = { 0x86, 0xDD };
	IPv4_ARP_t *parp_hdr;
	IPv6_Nadv_t *pNadv_hdr;
	t_u8 ret = MFALSE;

	/* IPV4 pkt check * A gratuitous ARP is an ARP packet * where the
	   source and destination IP are both set to * the IP of the machine
	   issuing the packet. */
	if (memcmp
	    (pmadapter, proto_ARP_type, &prx_pkt->eth803_hdr.h803_len,
	     sizeof(proto_ARP_type)) == 0) {
		parp_hdr = (IPv4_ARP_t *) (&prx_pkt->rfc1042_hdr);
		/* Graguitous ARP can be ARP request or ARP reply */
		if ((parp_hdr->op_code == mlan_htons(0x01)) ||
		    (parp_hdr->op_code == mlan_htons(0x02)))
			if (memcmp
			    (pmadapter, parp_hdr->src_ip, parp_hdr->dst_ip,
			     4) == 0)
				ret = MTRUE;
	}

	/* IPV6 pkt check * An unsolicited Neighbor Advertisement pkt is *
	   marked by a cleared Solicited Flag */
	if (memcmp
	    (pmadapter, proto_ARP_type_v6, &prx_pkt->eth803_hdr.h803_len,
	     sizeof(proto_ARP_type_v6)) == 0) {
		pNadv_hdr = (IPv6_Nadv_t *) (&prx_pkt->rfc1042_hdr);
		/* Check Nadv type: next header is ICMPv6 and icmp type is Nadv
		 */
		if (pNadv_hdr->next_hdr == 0x3A && pNadv_hdr->icmp_type == 0x88)
			if ((pNadv_hdr->flags & mlan_htonl(0x40000000)) == 0)
				ret = MTRUE;
	}

	return ret;
}

/**
 *  @brief This function processes received packet and forwards it
 *          to kernel/upper layer
 *
 *  @param pmadapter A pointer to mlan_adapter
 *  @param pmbuf   A pointer to mlan_buffer which includes the received packet
 *
 *  @return        MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status
wlan_process_rx_packet(pmlan_adapter pmadapter, pmlan_buffer pmbuf)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	pmlan_private priv = pmadapter->priv[pmbuf->bss_index];
	RxPacketHdr_t *prx_pkt;
	RxPD *prx_pd;
	int hdr_chop;
	EthII_Hdr_t *peth_hdr;
	t_u8 rfc1042_eth_hdr[MLAN_MAC_ADDR_LENGTH] = {
		0xaa, 0xaa, 0x03, 0x00, 0x00, 0x00
	};
	t_u8 snap_oui_802_h[MLAN_MAC_ADDR_LENGTH] = {
		0xaa, 0xaa, 0x03, 0x00, 0x00, 0xf8
	};
	t_u8 appletalk_aarp_type[2] = { 0x80, 0xf3 };
	t_u8 ipx_snap_type[2] = { 0x81, 0x37 };

	ENTER();

	prx_pd = (RxPD *) (pmbuf->pbuf + pmbuf->data_offset);
	prx_pkt = (RxPacketHdr_t *) ((t_u8 *) prx_pd + prx_pd->rx_pkt_offset);

/** Small debug type */
#define DBG_TYPE_SMALL  2
/** Size of debugging structure */
#define SIZE_OF_DBG_STRUCT 4
	if (prx_pd->rx_pkt_type == PKT_TYPE_DEBUG) {
		t_u8 dbg_type;
		dbg_type = *(t_u8 *) & prx_pkt->eth803_hdr;
		if (dbg_type == DBG_TYPE_SMALL) {
			PRINTM(MFW_D, "\n");
			DBG_HEXDUMP(MFW_D, "FWDBG",
				    (char *)((t_u8 *) & prx_pkt->eth803_hdr +
					     SIZE_OF_DBG_STRUCT),
				    prx_pd->rx_pkt_length);
			PRINTM(MFW_D, "FWDBG::\n");
		}
		goto done;
	}

	PRINTM(MINFO,
	       "RX Data: data_len - prx_pd->rx_pkt_offset = %d - %d = %d\n",
	       pmbuf->data_len, prx_pd->rx_pkt_offset,
	       pmbuf->data_len - prx_pd->rx_pkt_offset);

	HEXDUMP("RX Data: Dest", prx_pkt->eth803_hdr.dest_addr,
		sizeof(prx_pkt->eth803_hdr.dest_addr));
	HEXDUMP("RX Data: Src", prx_pkt->eth803_hdr.src_addr,
		sizeof(prx_pkt->eth803_hdr.src_addr));

	if ((memcmp(pmadapter, &prx_pkt->rfc1042_hdr,
		    snap_oui_802_h, sizeof(snap_oui_802_h)) == 0) ||
	    ((memcmp(pmadapter, &prx_pkt->rfc1042_hdr,
		     rfc1042_eth_hdr, sizeof(rfc1042_eth_hdr)) == 0) &&
	     memcmp(pmadapter, &prx_pkt->rfc1042_hdr.snap_type,
		    appletalk_aarp_type, sizeof(appletalk_aarp_type)) &&
	     memcmp(pmadapter, &prx_pkt->rfc1042_hdr.snap_type,
		    ipx_snap_type, sizeof(ipx_snap_type)))) {
		/*
		 * Replace the 803 header and rfc1042 header (llc/snap) with an
		 * EthernetII header, keep the src/dst and snap_type (ethertype).
		 * The firmware only passes up SNAP frames converting
		 * all RX Data from 802.11 to 802.2/LLC/SNAP frames.
		 * To create the Ethernet II, just move the src, dst address
		 * right before the snap_type.
		 */
		peth_hdr = (EthII_Hdr_t *)
			((t_u8 *) & prx_pkt->eth803_hdr
			 + sizeof(prx_pkt->eth803_hdr) +
			 sizeof(prx_pkt->rfc1042_hdr)
			 - sizeof(prx_pkt->eth803_hdr.dest_addr)
			 - sizeof(prx_pkt->eth803_hdr.src_addr)
			 - sizeof(prx_pkt->rfc1042_hdr.snap_type));

		memcpy(pmadapter, peth_hdr->src_addr,
		       prx_pkt->eth803_hdr.src_addr,
		       sizeof(peth_hdr->src_addr));
		memcpy(pmadapter, peth_hdr->dest_addr,
		       prx_pkt->eth803_hdr.dest_addr,
		       sizeof(peth_hdr->dest_addr));

		/* Chop off the RxPD + the excess memory from the
		   802.2/llc/snap header that was removed. */
		hdr_chop = (t_u32) ((t_ptr) peth_hdr - (t_ptr) prx_pd);
	} else {
		HEXDUMP("RX Data: LLC/SNAP",
			(t_u8 *) & prx_pkt->rfc1042_hdr,
			sizeof(prx_pkt->rfc1042_hdr));
		if ((priv->hotspot_cfg & HOTSPOT_ENABLED) &&
		    discard_gratuitous_ARP_msg(prx_pkt, pmadapter)) {
			ret = MLAN_STATUS_SUCCESS;
			PRINTM(MDATA,
			       "Bypass sending Gratuitous ARP frame to Kernel.\n");
			goto done;
		}
		/* Chop off the RxPD */
		hdr_chop =
			(t_u32) ((t_ptr) & prx_pkt->eth803_hdr -
				 (t_ptr) prx_pd);
	}

	/* Chop off the leading header bytes so the it points to the start of
	   either the reconstructed EthII frame or the 802.2/llc/snap frame */
	pmbuf->data_len -= hdr_chop;
	pmbuf->data_offset += hdr_chop;
	pmbuf->pparent = MNULL;

	DBG_HEXDUMP(MDAT_D, "RxPD", (t_u8 *) prx_pd,
		    MIN(sizeof(RxPD), MAX_DATA_DUMP_LEN));
	DBG_HEXDUMP(MDAT_D, "Rx Payload",
		    ((t_u8 *) prx_pd + prx_pd->rx_pkt_offset),
		    MIN(prx_pd->rx_pkt_length, MAX_DATA_DUMP_LEN));
	priv->rxpd_rate = prx_pd->rx_rate;
	priv->rxpd_htinfo = prx_pd->ht_info;

	pmadapter->callbacks.moal_get_system_time(pmadapter->pmoal_handle,
						  &pmbuf->out_ts_sec,
						  &pmbuf->out_ts_usec);
	PRINTM_NETINTF(MDATA, priv);
	PRINTM(MDATA, "%lu.%06lu : Data => kernel seq_num=%d tid=%d\n",
	       pmbuf->out_ts_sec, pmbuf->out_ts_usec, prx_pd->seq_num,
	       prx_pd->priority);

	ret = pmadapter->callbacks.moal_recv_packet(pmadapter->pmoal_handle,
						    pmbuf);
	if (ret == MLAN_STATUS_FAILURE) {
		pmbuf->status_code = MLAN_ERROR_PKT_INVALID;
		PRINTM(MERROR,
		       "STA Rx Error: moal_recv_packet returned error\n");
	}
done:
	if (ret != MLAN_STATUS_PENDING)
		wlan_free_mlan_buffer(pmadapter, pmbuf);
	LEAVE();

	return ret;
}

/**
 *   @brief This function processes the received buffer
 *
 *   @param adapter A pointer to mlan_adapter
 *   @param pmbuf     A pointer to the received buffer
 *
 *   @return        MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status
wlan_ops_sta_process_rx_packet(IN t_void * adapter, IN pmlan_buffer pmbuf)
{
	pmlan_adapter pmadapter = (pmlan_adapter) adapter;
	mlan_status ret = MLAN_STATUS_SUCCESS;
	RxPD *prx_pd;
	RxPacketHdr_t *prx_pkt;
	pmlan_private priv = pmadapter->priv[pmbuf->bss_index];
	t_u8 ta[MLAN_MAC_ADDR_LENGTH];
	t_u16 rx_pkt_type = 0;
	wlan_mgmt_pkt *pmgmt_pkt_hdr = MNULL;
	ENTER();

	prx_pd = (RxPD *) (pmbuf->pbuf + pmbuf->data_offset);
	/* Endian conversion */
	endian_convert_RxPD(prx_pd);
	rx_pkt_type = prx_pd->rx_pkt_type;
	prx_pkt = (RxPacketHdr_t *) ((t_u8 *) prx_pd + prx_pd->rx_pkt_offset);

	if ((prx_pd->rx_pkt_offset + prx_pd->rx_pkt_length) >
	    (t_u16) pmbuf->data_len) {
		PRINTM(MERROR,
		       "Wrong rx packet: len=%d,rx_pkt_offset=%d,"
		       " rx_pkt_length=%d\n", pmbuf->data_len,
		       prx_pd->rx_pkt_offset, prx_pd->rx_pkt_length);
		pmbuf->status_code = MLAN_ERROR_PKT_SIZE_INVALID;
		ret = MLAN_STATUS_FAILURE;
		wlan_free_mlan_buffer(pmadapter, pmbuf);
		goto done;
	}
	pmbuf->data_len = prx_pd->rx_pkt_offset + prx_pd->rx_pkt_length;

	if (pmadapter->priv[pmbuf->bss_index]->mgmt_frame_passthru_mask &&
	    prx_pd->rx_pkt_type == PKT_TYPE_MGMT_FRAME) {
		/* Check if this is mgmt packet and needs to forwarded to app
		   as an event */
		pmgmt_pkt_hdr =
			(wlan_mgmt_pkt *) ((t_u8 *) prx_pd +
					   prx_pd->rx_pkt_offset);
		pmgmt_pkt_hdr->frm_len =
			wlan_le16_to_cpu(pmgmt_pkt_hdr->frm_len);

		if ((pmgmt_pkt_hdr->wlan_header.frm_ctl
		     & IEEE80211_FC_MGMT_FRAME_TYPE_MASK) == 0)
			wlan_process_802dot11_mgmt_pkt(pmadapter->
						       priv[pmbuf->bss_index],
						       (t_u8 *) &
						       pmgmt_pkt_hdr->
						       wlan_header,
						       pmgmt_pkt_hdr->frm_len +
						       sizeof(wlan_mgmt_pkt)
						       -
						       sizeof(pmgmt_pkt_hdr->
							      frm_len));
		wlan_free_mlan_buffer(pmadapter, pmbuf);
		goto done;
	}

	/*
	 * If the packet is not an unicast packet then send the packet
	 * directly to os. Don't pass thru rx reordering
	 */
	if (!IS_11N_ENABLED(priv) ||
	    memcmp(priv->adapter, priv->curr_addr,
		   prx_pkt->eth803_hdr.dest_addr, MLAN_MAC_ADDR_LENGTH)) {
		wlan_process_rx_packet(pmadapter, pmbuf);
		goto done;
	}

	if (queuing_ra_based(priv)) {
		memcpy(pmadapter, ta, prx_pkt->eth803_hdr.src_addr,
		       MLAN_MAC_ADDR_LENGTH);
	} else {
		if ((rx_pkt_type != PKT_TYPE_BAR) &&
		    (prx_pd->priority < MAX_NUM_TID))
			priv->rx_seq[prx_pd->priority] = prx_pd->seq_num;
		memcpy(pmadapter, ta,
		       priv->curr_bss_params.bss_descriptor.mac_address,
		       MLAN_MAC_ADDR_LENGTH);
	}

	if ((priv->port_ctrl_mode == MTRUE && priv->port_open == MFALSE) &&
	    (rx_pkt_type != PKT_TYPE_BAR)) {
		mlan_11n_rxreorder_pkt(priv, prx_pd->seq_num, prx_pd->priority,
				       ta, (t_u8) prx_pd->rx_pkt_type,
				       (t_void *) RX_PKT_DROPPED_IN_FW);
		wlan_process_rx_packet(pmadapter, pmbuf);
		goto done;
	}

	/* Reorder and send to OS */
	ret = mlan_11n_rxreorder_pkt(priv, prx_pd->seq_num,
				     prx_pd->priority, ta,
				     (t_u8) prx_pd->rx_pkt_type, (void *)pmbuf);
	if (ret || (rx_pkt_type == PKT_TYPE_BAR)
		) {
		wlan_free_mlan_buffer(pmadapter, pmbuf);
	}

done:

	LEAVE();
	return ret;
}
