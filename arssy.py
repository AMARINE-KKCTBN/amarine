# import array




id_C = [1,2,3,4,5,10,11,12,13,20,21,22,23]
id_O = [6,7,8,9]
id_N = [14,15,16,17,18,19]
arr = [id_C,id_O,id_N]
beginning = 0
current = beginning
last = current

# while len(id_C):
beginning = arr[0][0]
for i in id_C:
    current = i
    if current - last == 1:
        last = current 
    else: 
        break

print(str(beginning) + "/" + str(last))
out = ''
for i in range(beginning, last+1, 1):
    print(i)
    id_C.remove(i)
    out = out + str(i)
    out = out + " "
print(out)
print(id_C)
print(len(arr))