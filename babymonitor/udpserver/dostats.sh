 ./stats out.raw | grep Delta7 |  awk 'BEGIN { DELTA7=0; TOTAL=0; } { DELTA7=DELTA7+; TOTAL+=1400; printf("%d/%d = %f%%\n", DELTA7, TOTAL, 100*DELTA7/TOTAL); }'

