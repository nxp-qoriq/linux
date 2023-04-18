#! /bin/bash
# SPDX-License-Identifier: GPL-2.0
# Copyright 2022-2023 NXP

set -eo pipefail

usage()
{
	echo "USAGE: $0 [-h] [-s pkt_size] [-r regression] [-t type] [-b backend copy] [-f frontend copy]"
	echo -e "-s: Pagket size: max 2048 Bytes, default: 64 Bytes"
	echo -e "-r: Regression times: default: 1000"
	echo -e "-t: Test type: 0: TX (frontend to backend); 1: RX (backend to frontend)"
	echo -e "-b: Backend copy buffer option:  0: not copy; 1: copy"
	echo -e "-f: Frontend copy buffer option: 0: not copy; 1: copy"
	echo -e "-h: This USAGE info"
	exit 1
}

setvar()
{
	local varname=$1
	shift
	if [ -z "${varname}" ]; then
		usage
	else
		eval "$varname=\"$@\""
	fi
}

detect_machine ()
{
	if grep -q 'i.MX8MM' /sys/devices/soc0/soc_id; then
		VIRTIO=b8400000
	elif grep -q 'i.MX8MP' /sys/devices/soc0/soc_id; then
		VIRTIO=b8400000
	elif grep -q 'i.MX93' /sys/devices/soc0/soc_id; then
		VIRTIO=a8400000
	fi
}

while getopts 'hs:t:r:b:f:' c
do
	case $c in
		h) usage ;;
		s) setvar PKT_SIZE $OPTARG ;;
		t) setvar TYPE $OPTARG ;;
		r) setvar REGRESS $OPTARG ;;
		b) setvar BACK_COPY $OPTARG ;;
		f) setvar FRONT_COPY $OPTARG ;;
	esac

done

if [ -z "${TYPE}" ]; then
	TYPE=0;
fi

if [ -z "${PKT_SIZE}" ]; then
	PKT_SIZE=64;
fi

if [ -z "${REGRESS}" ]; then
	REGRESS=1000;
fi

if [ -z "${BACK_COPY}" ]; then
	BACK_COPY=0;
fi

if [ -z "${FRONT_COPY}" ]; then
	FRONT_COPY=0;
fi

CONFIG=$(( $(( TYPE << 0 )) | $(( BACK_COPY << 1 )) | $(( FRONT_COPY << 2 )) ))

echo ${REGRESS} > /sys/devices/platform/${VIRTIO}.virtio_trans/virtio0/vt_regression&&
echo ${PKT_SIZE} > /sys/devices/platform/${VIRTIO}.virtio_trans/virtio0/vt_pkt_size&&
echo ${CONFIG} > /sys/devices/platform/${VIRTIO}.virtio_trans/virtio0/vt_config&&
echo 1 > /sys/devices/platform/${VIRTIO}.virtio_trans/virtio0/vt_control;
