#pragma once

#include "global_type.h"

void init_can_link(global_data *pdata);
int process_can_rx(global_data *pdata, CanardFrame *preceived_frame);
int decode_can_rx(global_data *pdata, CanardTransfer *ptransfer);
int tx_feed_back(global_data *pdata);
