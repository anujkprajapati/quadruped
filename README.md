# quadruped
Custom gym environment of a quadruped for RL training using SAC  

register using "pip install e ." in the quadruped root directory.  


make additional folders tmp/sac & plots:  
$ mkdir -p tmp/sac  
$ mkdir plots  

For training  
>>main_sac.py    
>>learning = True   
>>load_checkpoint = False       

For testing copy the model files in tmp/sac
>>main_sac.py  
>>learning = False
>>load_checkpoint = True
