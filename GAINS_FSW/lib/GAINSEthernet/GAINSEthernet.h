/*
 * GAINSEthernet.h
 *
 * Created on: 1 Mar. 2023
 * Author: Bennett Grow
 */

#ifndef _GAINSEthernet_H_
#define _GAINSEthernet_H_

#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

#ifdef __cplusplus
extern "C" {
#endif

void SSP_decode();
void UDP_send();

#ifdef __cplusplus
}
#endif

#endif