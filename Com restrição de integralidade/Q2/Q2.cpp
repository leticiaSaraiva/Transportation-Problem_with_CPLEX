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
IloObjective obj = IloMaximize(env);
IloCplex cplex(env);

void createModelAndSolve(){
	IloExpr exp(env);
	IloRangeArray con(env);
	
	IloNumVarArray y (env);
	IloNumVarArray z (env);

	for(int i=0; i < num_origem; i++){	
		char nome[20];
		sprintf(nome,"y%d",i+1);
		y.add(IloNumVar(env,0,IloInfinity,ILOINT));
	}
	
	for(int j=0; j < num_destino; j++){	
		char nome[20];
		sprintf(nome,"z%d",j+1);
		z.add(IloNumVar(env,0,IloInfinity,ILOINT));
	}
	
	
	for(int i=0;i<num_origem;i++){
		obj.setLinearCoef(y[i], -s_i[i]);
	}
	for(int j =0; j< num_destino;j++){
		obj.setLinearCoef(z[j], d_j[j]);
	}

	model.add(obj);
	
	
	for(int i=0;i<num_origem;i++){
		for(int j=0;j<num_destino;j++){
			IloExpr exp1(env);
			exp1 = -y[i] + z[j];
			exp1 -= c_i_j[i][j];
			
			con.add(IloRange(env, -IloInfinity, exp1, 0));
			exp1.clear();
		}
		
	}

	model.add(con);
    con.clear();
	cplex.extract(model);  		//Only for printing
    cplex.exportModel("PL.lp");   	//Only for printing



// --------------------- 
    double sol_val=0;
    double *Vy=NULL;

    Vy=(double*) malloc((num_origem)*sizeof(double));
    for(int i =0; i<num_origem;i++){
    	Vy[i]=0.0;
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
           	Vy[i] = cplex.getValue(y[i]);
        }
        result= true;
    }

    if(!result) printf("INFEASIBLE!\n");   // Return false if the master problem is infeasible

    printf("Objective Function Value = %.2lf\n",sol_val);
    
    free(Vy);
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




