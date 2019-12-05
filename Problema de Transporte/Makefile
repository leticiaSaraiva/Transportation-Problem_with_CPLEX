CC = g++
CFLAGS1 = -c -m64 -O -fPIC -fexceptions -DNDEBUG -DIL_STD -DILOSTRICTPOD
CFLAGS2 =    -m64 -O -O0 -fPIC -fexceptions -DNDEBUG -DIL_STD -DILOSTRICTPOD -Wall
LIBS =  -lm 
INCLUDES = -I/opt/ibm/ILOG/CPLEX_Studio1261/cplex/include/ -I/opt/ibm/ILOG/CPLEX_Studio1261/concert/include/
OBJS = Q1.o
SRCS = Q1.cpp
LIBS = -L/opt/ibm/ILOG/CPLEX_Studio1261/cplex/lib/x86-64_linux/static_pic -lilocplex -lcplex -L/opt/ibm/ILOG/CPLEX_Studio1261/concert/lib/x86-64_linux/static_pic -lconcert -lm -lpthread 
EXEC = Q1

compila: Q1.cpp
#para compilar no lia no cplex versão 11
	${CC} ${CFLAGS1} ${INCLUDES} ${SRCS}
	${CC} ${CFLAGS2} ${INCLUDES} ${OBJS} -o ${EXEC} ${LIBS}

executa: compila
	./Q1 ../instances/teste1.col
