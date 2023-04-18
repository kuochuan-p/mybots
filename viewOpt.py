from solution import SOLUTION

filename = input("Enter filename: ")
fnParts = filename.split(".")
id = fnParts[0][3:]
print(id)
f = open(filename, "r")

lines = f.readlines()
section = ""
weights = {"CPG":[], "Hidden":[], "Motor":[]}

numSensors =0
numHidden =0
numCPG =0
numMotors =0

for l in lines:
    l = l.strip()
    row = 0
    match l:
        case("Hidden weights:"):
            section = "Hidden"
        case("CPG weights:"):
            section = "CPG"
        case("Motor weights:"):
            section = "Motor"
        case _:
            splitLine = l.split(",")
            splitLine.pop(-1)
            for n in splitLine:
                n = float(n)
            weights[section].append(splitLine)


sol = SOLUTION(int(id), weights)
sol.Start_Simulation("GUI")



    