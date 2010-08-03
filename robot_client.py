#!/usr/bin/env python

import sys, struct, functools, socket

if __name__ == '__main__':
    position = []
    for pos in sys.argv[1:]:
        position.append(float(pos))
    npos = len(position)
    fmt = '=QQ%df' % npos
    print 'fmt', fmt
    msg = struct.pack(fmt, npos, 0, *position)
    print 'len(msg)', len(msg)
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    nbytes = udp.sendto(msg, ('localhost', 1382))
    print 'sent', nbytes, 'bytes'
