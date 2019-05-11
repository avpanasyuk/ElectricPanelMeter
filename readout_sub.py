# -*- coding: utf-8 -*-
"""
Created on Sat Feb  3 16:33:52 2018

@author: panasyuk
"""

import requests
import time
import datetime
import sys
import re

comerri = 0
numbers = list(map(chr,range(ord('0'),ord('9')+1))) + [',','.','-']
# lets see whether it is main or sub script
if len(sys.argv) < 2:
  print('USAGE: readout_{main|sub}.py configuration_version, '+
    'which should correspond to conf_{main|sub}.v?.m file in root directory')
  sys.exit()
try:
  type = re.search('readout_(.+?).py',sys.argv[0]).group(1)
  print(type)
except:
  print('Script name should be "readout_<type>.py", it is not!')

server_addr = 'http://ESP_06CD4D.home/read' if type == 'main' else 'http://ESP_06D157.home/read'

with requests.Session() as s:
  while True:
    time.sleep(5)
    try:
      response = s.get(server_addr,timeout=30) 
    except:
      print('Communication problem #' + str(comerri) + '\n')
      comerri = comerri + 1
      continue
    t = response.text
    open_ss = '<html>\r\n'
    close_ss = '<br><br></html>'
    t = t[t.find(open_ss) + len(open_ss):t.find(close_ss)]
    t = t.replace('<br>',',')
    for c in t:
      if c not in numbers:
        print('Error: ' + t)
        break
    else:
      print(t)
      CurMonth = datetime.datetime.now().strftime('%h')
      with open('/RAIDZ2/R/PowerMonitor.v' + sys.argv[1] + '.' + datetime.datetime.now().strftime('%h') +
        '.' + type + '.csv', 'a') as file:
        file.write(str(time.time()) + ',' + t + '\n') 
      
