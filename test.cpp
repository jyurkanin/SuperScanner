#include "super_scanner.h"
#include "scanner_window.h"
#include "audio_engine.h"

int main(int argc, char *argv[]){
    int num_nodes = 64;
    SuperScanner scanner(num_nodes);
    
    Vector3f X[num_nodes*2];
    Vector3f X1[num_nodes*2];
    
    scanner.rk4_temp = new Vector3f[num_nodes*2];
    scanner.k1 = new Vector3f[num_nodes*2];
    scanner.k2 = new Vector3f[num_nodes*2];
    scanner.k3 = new Vector3f[num_nodes*2];
    scanner.k4 = new Vector3f[num_nodes*2];

    
    scanner.strike();
    for(int i = 0; i < 1000; i++){
        scanner.solveRungeKutta(X, X1);

        
    }
  return 0;
}
