import numpy as np

arr = np.arange(8).reshape(4,2); 

f = open("test.h", "a")
f.truncate(0) ## Clears contents of file

f.write("static char str[] = {\n")


## SAVE SCALER ARRAY VALUES
np.savetxt(f,arr,
            fmt='"%.3f,"',
            delimiter='\t',
            comments='',
            header=f'"{arr.shape[0]},"\t"{arr.shape[1]},"',
            footer='"*"\n')

f.write("};")
f.close()