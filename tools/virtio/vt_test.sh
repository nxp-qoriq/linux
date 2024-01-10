#! /bin/bash
# SPDX-License-Identifier: GPL-2.0
# Copyright 2022-2024 NXP

set -eo pipefail

usage()
{
	echo "USAGE: $0 [-h] [-s pkt_size] [-r regression] [-t type] [-b backend copy] [-f frontend copy]"
	echo -e "-s: Packet size: max 2048 Bytes, default: 64 Bytes"
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

find_virtio_trans ()
{
	VIRTIO_TRANS=`find /sys/bus/platform/devices/ -name *.virtio_trans`
	if [ -z "${VIRTIO_TRANS}" ] || [ ! -d ${VIRTIO_TRANS} ]; then
		echo "${VIRTIO_TRANS}"
		exit 2;
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

find_virtio_trans

echo ${REGRESS} > ${VIRTIO_TRANS}/virtio0/vt_regression&&
echo ${PKT_SIZE} > ${VIRTIO_TRANS}/virtio0/vt_pkt_size&&
echo ${CONFIG} > ${VIRTIO_TRANS}/virtio0/vt_config&&
echo 1 > ${VIRTIO_TRANS}/virtio0/vt_control;
