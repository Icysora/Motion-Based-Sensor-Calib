
import numpy as np


# N = 100
# c = 0

# for i in range(0,100):
#     j = 2
#     k = (i * i) % 100
#     while k != i and j <= N:
#         k = (k * i) % 100 
#         j = j + 1
#     if j <= N:
#         print(">",i,":",j)
#         c = c + 1
#     else:
#         print(">",i,": *")
        
# print("> -- :",c)

A = np.random.rand(7,5)

print(np.linalg.svd(A,compute_uv=False))
        