#!/usr/bin/env python

import sys, struct, functools, socket, Tix, math

class Slider(Tix.Frame):
    def __init__(self, pframe, parent, name, index, unit, lower, upper, value):
        self.parent = parent
        self.name = name
        self.index = index
        Tix.Frame.__init__(self, pframe)
        self.unit_scale = 1
        if unit == 'rad' or unit == 'radians':
            unit = 'deg'
            self.unit_scale = math.pi / 180
        elif unit == 'm' or unit == 'meters':
            unit = 'mm'
            self.unit_scale = 0.001;
        lower /= self.unit_scale
        upper /= self.unit_scale
        value /= self.unit_scale
        labeltext = '%s [%s] (%5.2f to %5.2f)' % (name, unit, lower, upper)
        label = Tix.Label(self, text = labeltext, anchor = 'w')
        label.pack(side = Tix.TOP, expand = True, fill = Tix.X)
        self.var = Tix.DoubleVar(self)
        self.var.set(value)
        frame = Tix.Frame(self)
        Tix.Label(frame, text = 'value', width = 6).pack(side = Tix.LEFT)
        self.scale = Tix.Scale(frame, orient = Tix.HORIZONTAL,
                               variable = self.var, showvalue = 0,
                               from_ = lower, to = upper,
                               command = self._Update)
        self.scale.pack(side = Tix.LEFT, expand = True, fill = Tix.X)
        self.FORMAT = '%5.2f'
        self.label = Tix.Label(frame, text = self.FORMAT % value, width = 12)
        self.label.pack(side = Tix.LEFT)
        frame.pack(side = Tix.TOP, expand = True, fill = Tix.X)
        self.lower = lower
        self.upper = upper
        
    def _Update(self, value):
        self.label['text'] = self.FORMAT % float(value)
        self.parent.Notify(self.index, self.Get())
        
    def Get(self):
        return self.unit_scale * self.var.get()


class BunchOfSliders(Tix.Frame):
    def __init__(self, parent, ndof, host, port, **kwds):
        self.debug = False
        self.ready = False
        Tix.Frame.__init__(self, parent, kwds)
        self.ndof = ndof;
        self.ndata = 3 * ndof;
        self.fmt = '=QQQ%df' % self.ndata
        self.data = []
        for ii in range(ndof):
            frame = Tix.Frame(self)
            pos = Slider(frame, self, 'pos %d' % ii, ii, 'rad', -math.pi, math.pi, 0)
            pos.pack(side = Tix.LEFT, expand = True, fill = Tix.X)
            vel = Slider(frame, self, 'vel %d' % ii, ii + ndof, 'rad', -math.pi, math.pi, 0)
            vel.pack(side = Tix.LEFT, expand = True, fill = Tix.X)
            force = Slider(frame, self, 'com_delta %d' % ii, ii + 2 * ndof, '%', -100, 100, 0)
            force.pack(side = Tix.LEFT, expand = True, fill = Tix.X)
            frame.pack(side = Tix.TOP, expand = True, fill = Tix.X)
            self.data.append(0)
            self.data.append(0)
            self.data.append(0)
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.address = (host, port)
        self.ready = True
        
    def Notify(self, index, value):
        if not self.ready:
            return
        if (index < 0) or (index >= self.ndata):
            raise IndexError('index is %d but there are only %d NDATA' % (index, self.ndata))
        if index >= 2 * self.ndof:
            value = value / 100.0
        self.data[index] = value
        msg = struct.pack(self.fmt, self.ndof, self.ndof, self.ndof, *self.data)
        if self.debug:
            print 'sending:'
            print '  npos:', self.ndof
            print '  pos:', self.data[0:self.ndof]
            print '  nvel:', self.ndof
            print '  vel:', self.data[self.ndof:2*self.ndof]
            print '  nforce:', self.ndof
            print '  force:', self.data[2*self.ndof:3*self.ndof]
            print '  data:', repr(msg)
        nbytes = self.udp.sendto(msg, self.address)
        if len(msg) != nbytes:
            print 'OOPS, sent %d of %d bytes' % (nbytes, len(msg))


if __name__ == '__main__':
    if len(sys.argv) > 1:
        ndof = int(sys.argv[1])
    else:
        ndof = 4
    if ndof < 1:
        ndof = 1
    root = Tix.Tk()
    root.title('robot_client')
    bos = BunchOfSliders(root, ndof, 'localhost', 3589, bd = 2, relief = Tix.RIDGE)
    quitter = Tix.Button(root, text = 'QUIT', command = root.quit, bg = '#cc6666')
    bos.pack(side = Tix.TOP, expand = True, fill = Tix.X)
    quitter.pack(side = Tix.TOP, expand = True, fill = Tix.X)
    root.mainloop()
