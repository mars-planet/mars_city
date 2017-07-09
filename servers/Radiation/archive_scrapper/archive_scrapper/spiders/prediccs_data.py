from pymongo import MongoClient

count = 0
stack = []
flag = 0
flag2 = 0
Al_3_threshold = 
stack2 = []

for data in pre.find():
	flag2 = 0
	for i in data['data']:
		if not count or not stack:
			count = 1
			if float(i[-2]) > Al_3_threshold or flag == 1:
                stack.append(i)
                flag = 1
            if stack:
 		       	if float(i[-2]) < Al_3_threshold:
        			stack2.append(i)
        			if len(stack2) == clear_limit:
        				print stack
        				stack = []
        				stack2 = []
        else:
        	last_entry = stack[-1]
        	if last_entry == i:
        		flag2 = 1
        	if flag2:
        		if float(i[-2]) > Al_3_threshold or flag == 1:
                	stack.append(i)
                	flag = 1

        		if stack:
        			if float(i[-2]) < Al_3_threshold:
        				stack2.append(i)
        			if len(stack2) == clear_limit:
        				print stack
        				stack = []
        				stack2 = []
