#!/bin/bash

gcc -Wall -fcommon -o $1 $1".c" rpi_dma_utils.c
