from gurobipy import *

NUM_DAYS = 5
NUM_DESTINATIONS = 2

DESTINATION = ["AB", "AC", "BA", "BC", "CA", "CB"]
AIRPORTS = ["A", "B", "C"]

GRB_TYPE = GRB.CONTINUOUS

DEMAND = {
    "AB": [100, 200, 100, 400, 300],
    "AC": [50, 50, 50, 50, 50],
    "BA": [25, 25, 25, 25, 25],
    "BC": [25, 25, 25, 25, 25],
    "CA": [40, 40, 40, 40, 40],
    "CB": [400, 200, 300, 200, 400]
}

REPOSITIONING_COST = {
    "AB": 7,
    "AC": 3,
    "BA": 7,
    "BC": 6,
    "CA": 3,
    "CB": 6
}

MAX_NUM_AIRPLANES = 1200


# create a new model

myModel = Model("final_project")

# create decision variables

flight_dv = [dict() for x in range(NUM_DAYS)]

empty_flight_dv = [dict() for x in range(NUM_DAYS)]

stay_flight_dv = [dict() for x in range(NUM_DAYS)]

available_cargo_dv = [dict() for x in range(NUM_DAYS)]


# initialize decision variables

for day in range(NUM_DAYS):
    for destination in DESTINATION:
        flightVar = myModel.addVar(vtype=GRB_TYPE, name="flight_" + str(day) + "_" + str(destination))
        emptyVar = myModel.addVar(vtype=GRB_TYPE, name="empty_" + str(day) + "_" + str(destination))
        availableCargoVar = myModel.addVar(vtype=GRB_TYPE, name="available_cargo_" + str(day) + "_" + str(destination))
        flight_dv[day][destination] = flightVar
        empty_flight_dv[day][destination] = emptyVar
        available_cargo_dv[day][destination] = availableCargoVar


for day in range(NUM_DAYS):
    for airport in AIRPORTS:
        stayVar = myModel.addVar(vtype=GRB_TYPE, name="stay_" + str(day) + "_" + str(airport))
        stay_flight_dv[day][airport] = stayVar


# integrate decision variables into the model

myModel.update()

# create a linear expression for the objective

objExpr = LinExpr()

for day in range(NUM_DAYS):

    for from_airport in AIRPORTS:

        for to_airport in AIRPORTS:
            if from_airport != to_airport:
                destination = from_airport + to_airport
                objExpr += 10 * available_cargo_dv[day][destination] + REPOSITIONING_COST[destination] * empty_flight_dv[day][destination]


myModel.setObjective(objExpr, GRB.MINIMIZE)


# create constraints

# first constraint

for to_airport in AIRPORTS:
    leftExpr = LinExpr()
    for from_airport in AIRPORTS:
        if from_airport != to_airport:
            destination = from_airport + to_airport
            leftExpr += flight_dv[4][destination] + empty_flight_dv[4][destination]

    leftExpr += stay_flight_dv[4][to_airport]


myModel.addConstr(lhs=leftExpr, sense=GRB.EQUAL, rhs=MAX_NUM_AIRPLANES, name="t_1_" + str(4))


# second constraint

for day in range(NUM_DAYS):

    for to_airport in AIRPORTS:
        leftExpr = LinExpr()
        rightExpr = LinExpr()
        for from_airport in AIRPORTS:
            if from_airport != to_airport:
                destination = from_airport + to_airport
                alternate_destination = to_airport + from_airport
                leftExpr += flight_dv[day][destination] + empty_flight_dv[day][destination]
                if day + 1 <= 4:
                    rightExpr += flight_dv[day + 1][alternate_destination] + empty_flight_dv[day + 1][alternate_destination]
                else:
                    rightExpr += flight_dv[0][alternate_destination] + empty_flight_dv[0][alternate_destination]

        leftExpr += stay_flight_dv[day][to_airport]
        if day + 1 <= 4:
            rightExpr += stay_flight_dv[day + 1][to_airport]
        else:
            rightExpr += stay_flight_dv[0][to_airport]

        myModel.addConstr(lhs=leftExpr, sense=GRB.EQUAL, rhs=rightExpr, name="t_2_" + str(day) + '_' + to_airport)


# third constraint

for day in range(NUM_DAYS):
    for from_airport in AIRPORTS:
        leftExpr = LinExpr()
        rightExpr = LinExpr()
        for to_airport in AIRPORTS:

            if from_airport != to_airport:
                destination = from_airport + to_airport
                leftExpr = available_cargo_dv[day][destination]
                if day - 1 >= 0:
                    rightExpr += available_cargo_dv[day - 1][destination] + DEMAND[destination][day] - \
                                 flight_dv[day][destination]
                else:
                    rightExpr += available_cargo_dv[4][destination] + DEMAND[destination][day] - \
                                 flight_dv[day][destination]

        myModel.addConstr(lhs=leftExpr, sense=GRB.EQUAL, rhs=rightExpr, name="t_3_" + str(day) + '_' + from_airport)


# fourth constraint

for day in range(NUM_DAYS):
    for from_airport in AIRPORTS:
        for to_airport in AIRPORTS:
            leftExpr = LinExpr()
            rightExpr = LinExpr()
            if from_airport != to_airport:
                destination = from_airport + to_airport
                leftExpr = flight_dv[day][destination]
                if day - 1 >= 0:
                    rightExpr = available_cargo_dv[day - 1][destination] + DEMAND[destination][day]
                else:
                    rightExpr = available_cargo_dv[4][destination] + DEMAND[destination][day]

            myModel.addConstr(lhs=leftExpr, sense=GRB.LESS_EQUAL, rhs=rightExpr, name="t_4_" + str(day) + '_' + destination)


myModel.update()

# write the model in a file to make sure it is constructed correctly

myModel.write(filename="project.lp")

# optimize the model

myModel.optimize()

# print optimal objective and optimal solution

print("\nOptimal Objective: " + str(myModel.ObjVal))
print("\nOptimal Solution:")

allVars = myModel.getVars()

for curVar in allVars:
    print (curVar.varName + " " + str(curVar.x))
