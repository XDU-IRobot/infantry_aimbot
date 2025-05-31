#
# Copyright (c) 2025 XDU-IRobot
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

if(NOT WIN32)
  string(ASCII 27 Esc)
  set(ColourReset "${Esc}[m")
  set(ColourBold "${Esc}[1m")
  set(ColourRed "${Esc}[31m")
  set(ColourGreen "${Esc}[32m")
  set(ColourYellow "${Esc}[33m")
  set(ColourBlue "${Esc}[34m")
  set(ColourMagenta "${Esc}[35m")
  set(ColourCyan "${Esc}[36m")
  set(ColourWhite "${Esc}[37m")
  set(ColourBoldRed "${Esc}[1;31m")
  set(ColourBoldGreen "${Esc}[1;32m")
  set(ColourBoldYellow "${Esc}[1;33m")
  set(ColourBoldBlue "${Esc}[1;34m")
  set(ColourBoldMagenta "${Esc}[1;35m")
  set(ColourBoldCyan "${Esc}[1;36m")
  set(ColourBoldWhite "${Esc}[1;37m")
endif()
