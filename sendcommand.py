#!/usr/bin/python

from __future__ import print_function
import sys
import telnetlib
import argparse

#Telnet response timeout
timeout = 0.5

# Command line argument interface
parser = argparse.ArgumentParser(
    description="Stream g-code commands to Smoothieboard via telnet.")
parser.add_argument(
    '-i', '--ipaddr', help="Smoothieboard IP address", action='store')
parser.add_argument('-p', '--port', action='store', default=23)
parser.add_argument('cmd', action='store')

args = parser.parse_args()

# Stream command to Smoothieboard
tn = telnetlib.Telnet(args.ipaddr, args.port, timeout=10)
# Read startup prompt
tn.read_until(">")
tn.write(args.cmd)
ret = tn.read_eager()
if ret.count("ok") == 0:
    print("Ackowledgement of command " + args.cmd +
          " not received from Smoothieboard.\n")

tn.write("exit\n")
