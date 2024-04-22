#include"test.h"

using namespace std;

//void Test::test_gurobi() {
//    try {
//        // Create environment and model
//        GRBEnv env = GRBEnv();
//        GRBModel model = GRBModel(env);
//
//        // Create variables
//        GRBVar x = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "x");
//        GRBVar y = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "y");
//
//        // Set objective function
//        GRBLinExpr obj = x + y;
//        model.setObjective(obj, GRB_MAXIMIZE);
//
//        // Add constraints
//        model.addConstr(2 * x + y <= 10, "c1");
//        model.addConstr(x + 3 * y <= 12, "c2");
//
//        // Optimize model
//        model.optimize();
//
//        // Print solution
//        cout << "Objective value: " << model.get(GRB_DoubleAttr_ObjVal) << endl;
//        cout << "x = " << x.get(GRB_DoubleAttr_X) << endl;
//        cout << "y = " << y.get(GRB_DoubleAttr_X) << endl;
//
//    }
//    catch (GRBException e) {
//        cout << "Error code = " << e.getErrorCode() << endl;
//        cout << e.getMessage() << endl;
//    }
//    catch (...) {
//        cout << "Error during optimization" << endl;
//    }
//}
