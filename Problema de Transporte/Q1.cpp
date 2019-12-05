//Letícia Saraiva

#include <stdio.h>
#include <time.h>
#include <iostream>
#include <limits.h>
#include <math.h>
#include <ilcplex/ilocplex.h>
#include <vector>
#include <list>
#include <malloc.h>

ILOSTLBEGIN
using namespace std;
int num_origem,num_destino;

int c_i_j[100][100];
int d_j[100];
int s_i[100];


IloEnv env;
IloModel model(env);
IloObjective obj = IloMinimize(env);
IloCplex cplex(env);

void createModelAndSolve(){
	IloExpr exp(env);
	IloRangeArray con(env);
	IloNumVarArray  var_x [num_origem];

	for(int i=0; i < num_origem; i++){
		var_x[i] = IloNumVarArray(env);

		for(int j=0; j< num_destino; j++){
			char nome[20];
            sprintf(nome,"x%d_%d",i+1,j+1);
			var_x[i].add(IloNumVar(env,0,IloInfinity,ILOFLOAT));
		}
	}

	for(int i=0;i<num_origem;i++){
		for(int j =0; j< num_destino;j++){
			obj.setLinearCoef(var_x[i][j], c_i_j[i][j]);
		}
	}

	model.add(obj);
	
	for(int i=0;i<num_origem;i++){
		IloExpr exp1(env);
		for(int j=0;j<num_destino;j++){
			exp1 += var_x[i][j];
		}
		exp1 -= s_i[i];
		
		con.add(IloRange(env, -IloInfinity, exp1, 0));
		exp1.clear();
		
	}
	
    for(int j=0;j<num_destino;j++){
		IloExpr exp2(env);
		for(int i=0;i<num_origem;i++){
			exp2 += var_x[i][j];
		}
		exp2 -= d_j[j];
		
		con.add(IloRange(env, 0, exp2, IloInfinity));
		exp2.clear();
	}	

	model.add(con);
    con.clear();
	cplex.extract(model);  		//Only for printing
    cplex.exportModel("PL.lp");   	//Only for printing




// --------------------- 
    double sol_val=1;
    double **x=NULL;

    x=(double**) malloc((num_origem)*sizeof(double*));
    for(int i =0; i<num_origem;i++){
    	x[i]=(double*) malloc((num_origem)*sizeof(double));


    }	
	
    if ( !cplex.solve() && cplex.getStatus() != IloAlgorithm::Infeasible) {
        env.error() << "Failed to optimize LP" << endl;
        cout << cplex.getStatus() << endl;
        throw(-1);
    }

    // Get solution
    bool result = false ; 
    if(cplex.getStatus() != IloAlgorithm::Infeasible) {
        sol_val = cplex.getObjValue();
        for(int i=0;i<num_origem;i++){
            for(int j=0;j<num_destino;j++){
            	x[i][j] = cplex.getValue(var_x[i][j]);
            }
        }
        result= true;
    }

    if(!result) printf("INFEASIBLE!\n");   // Return false if the master problem is infeasible

    printf("Objective Function Value = %.2lf\n",sol_val);
    for(int i=0;i<num_origem;i++){
		for( int j = 0;j<num_destino;j++){
			printf("x[%d][%d] = %2.lf\n", i+1,j+1, x[i][j]);
		}
	}
    free(x);
}


void Read(FILE * transp){
	printf("Reading lines...\n");

	char type = fgetc(transp);
	
	if(type == 'i'){
		fscanf(transp,"%d %d\n",&num_origem, &num_destino);
	}

	type = fgetc(transp);
	if(type = 'c'){
		for (int i = 0; i < num_origem ;i++) {
			for (int j = 0; j < num_destino && type == 'c'; j++){
				fscanf(transp, "%d\n", &c_i_j[i][j]);
				//printf("%d\n", c_i_j[i][j]);
				type = fgetc(transp);
			}
		}
	}

	for (int i = 0; i < num_origem && type == 'o'; i++){
		fscanf(transp, "%d\n", &s_i[i]);
		//printf("%d\n", s_i[i]);
		type = fgetc(transp);
	}

	for (int j = 0; j < num_destino && type == 'd'; j++){
		fscanf(transp, "%d\n", &d_j[j]);
		//printf("%d\n", d_j[j]);
		type = fgetc(transp);
	}


}



int main(int argc, char * argv[]){

	if (argc < 2) {
        printf("Please, specify the file.\n");
        return 0;
    }
    FILE * pFile = fopen(argv[1], "r");
    if (pFile == NULL) {
        printf("Could not open %s.\n", argv[1]);
        return 0;
    }
	Read(pFile);
	fclose(pFile);



	clock_t start, end;
    double elapsed;
    start = clock();
    // Solving the problem
    cplex.setOut(env.getNullStream());      // Do not print cplex informations
    cplex.setWarning(env.getNullStream());  // Do not print cplex warnings
    createModelAndSolve();
    

    // Free memory

    // Getting run time
    //env.end();
    end = clock();
    elapsed = ((double) (end - start)) / CLOCKS_PER_SEC;
    printf("Time: %.5g second(s).\n", elapsed);

    return 0;
}




