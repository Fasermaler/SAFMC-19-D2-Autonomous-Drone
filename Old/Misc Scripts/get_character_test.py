import readchar
print("Reading a char:")
gettedchar = repr(readchar.readchar())
if gettedchar == "'a'":
    print("gotted a")
print("Reading a key:")
print(repr(readchar.readkey()))