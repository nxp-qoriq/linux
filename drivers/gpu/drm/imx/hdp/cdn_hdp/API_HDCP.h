/******************************************************************************
 *
 * Copyright (C) 2016-2017 Cadence Design Systems, Inc.
 * All rights reserved worldwide.
 *
 * Copyright 2017-2018 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ******************************************************************************
 *
 * API_HDCP.h
 *
 ******************************************************************************
 */

#ifndef _API_HDCP_H_
#define _API_HDCP_H_

/**
 * \addtogroup HDCP_API
 * \{
 */

#include "API_General.h"
#include "hdcp_tran.h"

/**
 * \brief send HDCP_TX_CONFIGURATION command
 * \return status
 */
CDN_API_STATUS CDN_API_HDCP_TX_CONFIGURATION(state_struct *state, u8 val,
					     CDN_BUS_TYPE bus_type);

/**
 * \brief blocking version of #CDN_API_HDCP_TX_CONFIGURATION
 */
CDN_API_STATUS CDN_API_HDCP_TX_CONFIGURATION_blocking(state_struct *state,
						      u8 val,
						      CDN_BUS_TYPE bus_type);

/**
 * \brief send HDCP2_TX_SET_PUBLIC_KEY_PARAMS command
 * \return status
 */
CDN_API_STATUS
CDN_API_HDCP2_TX_SET_PUBLIC_KEY_PARAMS(state_struct *state,
				       S_HDCP_TRANS_PUBLIC_KEY_PARAMS *val,
				       CDN_BUS_TYPE bus_type);

/**
 * \brief blocking version of #CDN_API_HDCP2_TX_SET_PUBLIC_KEY_PARAMS
 */
CDN_API_STATUS
CDN_API_HDCP2_TX_SET_PUBLIC_KEY_PARAMS_blocking(state_struct *state,
						S_HDCP_TRANS_PUBLIC_KEY_PARAMS *val,
						CDN_BUS_TYPE bus_type);

/**
 * \brief send HDCP2_TX_SET_KM_KEY_PARAMS command
 * \return status
 */
CDN_API_STATUS CDN_API_HDCP2_TX_SET_KM_KEY_PARAMS(state_struct *state,
						  S_HDCP_TRANS_KM_KEY_PARAMS *val,
						  CDN_BUS_TYPE bus_type);

/**
 * \brief blocking version of #CDN_API_HDCP2_TX_SET_KM_KEY_PARAMS
 */
CDN_API_STATUS
CDN_API_HDCP2_TX_SET_KM_KEY_PARAMS_blocking(state_struct *state,
					    S_HDCP_TRANS_KM_KEY_PARAMS *val,
					    CDN_BUS_TYPE bus_type);

/**
 * \brief send HDCP2_TX_SET_DEBUG_RANDOM_NUMBERS command
 * \return status
 */
CDN_API_STATUS
CDN_API_HDCP2_TX_SET_DEBUG_RANDOM_NUMBERS(state_struct *state,
					  S_HDCP_TRANS_DEBUG_RANDOM_NUMBERS *val,
					  CDN_BUS_TYPE bus_type);

/**
 * \brief blocking version of #CDN_API_HDCP2_TX_SET_DEBUG_RANDOM_NUMBERS
 */
CDN_API_STATUS
	CDN_API_HDCP2_TX_SET_DEBUG_RANDOM_NUMBERS_blocking
	(state_struct *state, S_HDCP_TRANS_DEBUG_RANDOM_NUMBERS *val,
	CDN_BUS_TYPE bus_type);

/**
 * \brief send HDCP2_TX_RESPOND_KM command
 * \param val - if NULL no arguments will be send
 * \return status
 */
CDN_API_STATUS CDN_API_HDCP2_TX_RESPOND_KM(state_struct *state,
					   S_HDCP_TRANS_PAIRING_DATA *val,
					   CDN_BUS_TYPE bus_type);

/**
 * \brief blocking version of #CDN_API_HDCP2_TX_RESPOND_KM
 */
CDN_API_STATUS CDN_API_HDCP2_TX_RESPOND_KM_blocking(state_struct *state,
						    S_HDCP_TRANS_PAIRING_DATA *val,
							CDN_BUS_TYPE bus_type);

/**
 * \brief send HDCP1_TX_SEND_KEYS command
 * \return status
 */
CDN_API_STATUS
CDN_API_HDCP1_TX_SEND_KEYS(state_struct *state,
			   S_HDCP_TX_MAIL_BOX_CMD_HDCP1_TX_SEND_KEYS *val,
			   CDN_BUS_TYPE bus_type);

/**
 * \brief blocking version of #CDN_API_HDCP1_TX_SEND_KEYS
 */
CDN_API_STATUS
CDN_API_HDCP1_TX_SEND_KEYS_blocking(state_struct *state,
				    S_HDCP_TX_MAIL_BOX_CMD_HDCP1_TX_SEND_KEYS *val,
					CDN_BUS_TYPE bus_type);

/**
 * \brief send HDCP1_TX_SEND_RANDOM_AN command
 * \return status
 */
CDN_API_STATUS CDN_API_HDCP1_TX_SEND_RANDOM_AN(state_struct *state, u8 An[8],
					       CDN_BUS_TYPE bus_type);

/**
 * \brief blocking version of #CDN_API_HDCP1_TX_SEND_RANDOM_AN
 */
CDN_API_STATUS CDN_API_HDCP1_TX_SEND_RANDOM_AN_blocking(state_struct *state,
							u8 An[8],
							CDN_BUS_TYPE bus_type);

/**
 * \brief send HDCP_TX_STATUS_REQ command
 * \return status
 */
CDN_API_STATUS CDN_API_HDCP_TX_STATUS_REQ(state_struct *state, u8 resp[5],
					  CDN_BUS_TYPE bus_type);

/**
 * \brief blocking version of #CDN_API_HDCP_TX_STATUS_REQ
 */
CDN_API_STATUS CDN_API_HDCP_TX_STATUS_REQ_blocking(state_struct *state,
						   u8 resp[5],
						   CDN_BUS_TYPE bus_type);

/**
 * \brief send HDCP2_TX_IS_KM_STORED_REQ command
 * \return status
 */
CDN_API_STATUS CDN_API_HDCP2_TX_IS_KM_STORED_REQ(state_struct *state,
						 u8 resp[5],
						 CDN_BUS_TYPE bus_type);

/**
 * \brief blocking version of #CDN_API_HDCP2_TX_IS_KM_STORED_REQ
 */
CDN_API_STATUS CDN_API_HDCP2_TX_IS_KM_STORED_REQ_blocking(state_struct *state,
							  u8 resp[5],
							  CDN_BUS_TYPE
							  bus_type);

/**
 * \brief send HDCP2_TX_STORE_KM_REQ command
 * \return status
 */
CDN_API_STATUS CDN_API_HDCP2_TX_STORE_KM_REQ(state_struct *state,
					     S_HDCP_TRANS_PAIRING_DATA *resp,
					     CDN_BUS_TYPE bus_type);

/**
 * \brief blocking version of #CDN_API_HDCP2_TX_STORE_KM_REQ
 */
CDN_API_STATUS CDN_API_HDCP2_TX_STORE_KM_REQ_blocking(state_struct *state,
						      S_HDCP_TRANS_PAIRING_DATA
						      *resp,
						      CDN_BUS_TYPE bus_type);

/**
 * \brief send HDCP_TX_IS_RECEIVER_ID_VALID_REQ command
 * \return status
 */
CDN_API_STATUS CDN_API_HDCP_TX_IS_RECEIVER_ID_VALID_REQ(state_struct *state,
							u8 *num, u8 *id,
							CDN_BUS_TYPE bus_type);

/**
 * \brief blocking version of #CDN_API_HDCP_TX_IS_RECEIVER_ID_VALID_REQ
 */
CDN_API_STATUS CDN_API_HDCP_TX_IS_RECEIVER_ID_VALID_REQ_blocking(state_struct *state,
								 u8 *num,
								 u8 *id,
								 CDN_BUS_TYPE
								 bus_type);

/**
 * \brief send HDCP_TX_RESPOND_RECEIVER_ID_VALID command
 * \return status
 */
CDN_API_STATUS CDN_API_HDCP_TX_RESPOND_RECEIVER_ID_VALID(state_struct *state,
							 u8 valid,
							 CDN_BUS_TYPE bus_type);

/**
 * \brief blocking version of #CDN_API_HDCP_TX_RESPOND_RECEIVER_ID_VALID
 */
CDN_API_STATUS CDN_API_HDCP_TX_RESPOND_RECEIVER_ID_VALID_blocking(state_struct *state,
								  u8 valid,
								  CDN_BUS_TYPE
								  bus_type);

CDN_API_STATUS CDN_API_HDCP_GENERAL_2_SET_LC(state_struct *state, u8 *lc,
					     CDN_BUS_TYPE bus_type);
CDN_API_STATUS CDN_API_HDCP_GENERAL_2_SET_LC_blocking(state_struct *state,
						      u8 *lc,
						      CDN_BUS_TYPE bus_type);
/* TODO DK: Implement */
CDN_API_STATUS CDN_API_HDCP_SET_SEED(state_struct *state, u8 *seed,
				     CDN_BUS_TYPE bus_type);
CDN_API_STATUS CDN_API_HDCP_SET_SEED_blocking(state_struct *state, u8 *seed,
					      CDN_BUS_TYPE bus_type);
CDN_API_STATUS CDN_API_TEST_KEYS(state_struct *state, u8 test_type, u8 resp[1],
				 CDN_BUS_TYPE bus_type);
CDN_API_STATUS CDN_API_TEST_KEYS_blocking(state_struct *state, u8 test_type,
					  u8 resp[1], CDN_BUS_TYPE bus_type);

#endif
