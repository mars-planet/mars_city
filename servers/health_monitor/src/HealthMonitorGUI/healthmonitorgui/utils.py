from __future__ import print_function
import wx

def capture_window(window, file_name):
    print('Capturing screen {}'.format(window.Title))
    context = wx.WindowDC(window)
    memory = wx.MemoryDC()
    x, y = window.ScreenRect.Size
    bitmap = wx.EmptyBitmap(x, y, -1)
    memory.SelectObject(bitmap)
    memory.Blit(0, 0, x, y, context, 0, 0)
    memory.SelectObject(wx.NullBitmap)
    print('Saving to file {}'.format(file_name))
    bitmap.SaveFile(file_name, wx.BITMAP_TYPE_PNG)

