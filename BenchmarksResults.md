
# Methods:
## Filter()
*  min height = 4[ms]
*  stage 1 = 4[ms]
*  stage 2 = 860[ms] 
  

_______________________________________________________________________________

### Benchmark: filter  
Attempts: 5    
Duration: 5 seconds  
_______________________________________________________________________________  
Phase: filter  
Average time: 885.402 [ms/op]  
Minimal time: 885.402 [ms/op]  
Maximal time: 907.924 [ms/op]  
Total time: 4.427 [s]  
Total operations: 5  
Operations throughput: 1 [ops/s]  
_______________________________________________________________________________  

### Notes
Looks like stage 2: statistical outlier removal takes most of the time
test its outcome and if it is needed 