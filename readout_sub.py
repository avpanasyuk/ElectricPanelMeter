# -*- coding: utf-8 -*-
"""
Created on Sat Feb  3 16:33:52 2018

@author: panasyuk
"""

import requests
import time

comerri = 0
numbers = list(map(chr,range(ord('0'),ord('9')+1))) + [',','.','-']

with requests.Session() as s:
  while True:
    time.sleep(5)
    try:
      # response = s.get('http://ESP_06CD4D.home/read',timeout=30) # V1 - main
      response = s.get('http://ESP_06D157.home/read',timeout=30) # V2 - sub
    except:
      print('Communication problem #' + str(comerri) + '\n')
      comerri = comerri + 1
      continue
    t = response.text
    open_ss = '<html>\r\n'
    close_ss = '<br>\r\n</html>'
    t = t[t.find(open_ss) ++ len(open_ss):t.find(close_ss)]
    t = t.replace('<br>',',')
    for c in t:
      if c not in numbers:
        print('Error: ' + t)
        break
    else:
      print(t)
      with open('/RAIDZ2/R/PowerMonitor_main.csv', 'a') as file:
        file.write(str(time.time()) + ',' + t + '\n') 
      
