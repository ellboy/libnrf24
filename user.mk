USERLIB = $(CURDIR)
$(info VAR="$(USERLIB)")
# List of all the Userlib files
USERSRC =  $(USERLIB)/src/rf.c 
          
# Required include directories
USERINC =  $(USERLIB) \
           $(USERLIB)/include 
