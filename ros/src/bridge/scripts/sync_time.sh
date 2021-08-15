#!/bin/bash
ssh -t root@192.168.1.123 date --set $(date "+%Y-%m-%dT%H:%M:%S")
