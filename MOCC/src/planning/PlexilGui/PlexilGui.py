

#### Add the fields in line 351 to DB ####



import wx

from thread import start_new_thread
import string

class Frame(wx.Frame):
	def __init__(self,parent,id,title):
		super(Frame,self).__init__(parent,id,title,size=(900,900))

		hbox = wx.BoxSizer(wx.HORIZONTAL)
		vbox = wx.BoxSizer(wx.VERTICAL)
		
		self.a={}

		#panel1 for the tree representation part and panel2 for displaying the node characteristics when activated on the tree
		self.panel1 = wx.Panel(self, -1)
		self.panel2 = wx.Panel(self, -1)

		self.tree = wx.TreeCtrl(self.panel1, 1, wx.DefaultPosition, (-1,-1), wx.TR_HIDE_ROOT|wx.TR_HAS_BUTTONS)
		self.root = self.tree.AddRoot('Plan')
		self.a['root']=self.root

		self.Bind(wx.EVT_TREE_ITEM_ACTIVATED,self.OnActivated,self.tree)


		vbox2=wx.BoxSizer(wx.VERTICAL)
		vbox3=wx.BoxSizer(wx.VERTICAL)

		h1box=wx.BoxSizer(wx.HORIZONTAL)
		self.StaticText_Node_Type_List=wx.StaticText(self.panel2,-1,"Node Type :")
		h1box.Add(self.StaticText_Node_Type_List,1)
		self.StaticText_Node_Type_List1=wx.StaticText(self.panel2,-1,label='_')
		h1box.Add(self.StaticText_Node_Type_List1,1)

		vbox2.Add(h1box,1,wx.EXPAND)

		h7box=wx.BoxSizer(wx.HORIZONTAL)
		self.StaticText_Node_Parent=wx.StaticText(self.panel2,-1,"Node Parent :")
		h7box.Add(self.StaticText_Node_Parent,1)
		self.StaticText_Node_Parent1=wx.StaticText(self.panel2,-1,label='_')
		h7box.Add(self.StaticText_Node_Parent1,1)

		vbox3.Add(h7box,1,wx.EXPAND)
		
		h2box=wx.BoxSizer(wx.HORIZONTAL)
		self.StaticText_Node_Name=wx.StaticText(self.panel2,-1,"Node Name :")
		h2box.Add(self.StaticText_Node_Name,1)
		self.StaticText_Node_Name1=wx.StaticText(self.panel2,-1,label='_')
		h2box.Add(self.StaticText_Node_Name1,1)

		vbox2.Add(h2box,1,wx.EXPAND)

		h8box=wx.BoxSizer(wx.HORIZONTAL)
		self.StaticText_Node_Function=wx.StaticText(self.panel2,-1,'Node Function:')
		h8box.Add(self.StaticText_Node_Function,1)
		self.StaticText_Node_Function1=wx.StaticText(self.panel2,-1,label='_')
		h8box.Add(self.StaticText_Node_Function1,1)

		vbox3.Add(h8box,1,wx.EXPAND)

		h3box=wx.BoxSizer(wx.HORIZONTAL)
		self.StaticText_Start_C=wx.StaticText(self.panel2,-1,"Start Condition:")
		h3box.Add(self.StaticText_Start_C,1)
		self.StaticText_Start_C1=wx.StaticText(self.panel2,-1,label='_')
		h3box.Add(self.StaticText_Start_C1,1)

		vbox2.Add(h3box,1,wx.EXPAND)

		h9box=wx.BoxSizer(wx.HORIZONTAL)
		self.StaticText_Pre_C=wx.StaticText(self.panel2,-1,"Pre Condition:")
		h9box.Add(self.StaticText_Pre_C)
		self.StaticText_Pre_C1=wx.StaticText(self.panel2,-1,label='_')
		h9box.Add(self.StaticText_Pre_C1)

		vbox3.Add(h9box,1,wx.EXPAND)

		h4box=wx.BoxSizer(wx.HORIZONTAL)
		self.StaticText_Repeat_C=wx.StaticText(self.panel2,-1,"Repeat Condition :")
		h4box.Add(self.StaticText_Repeat_C,1)
		self.StaticText_Repeat_C1=wx.StaticText(self.panel2,-1,label='_')
		h4box.Add(self.StaticText_Repeat_C1,1)

		vbox2.Add(h4box,1,wx.EXPAND)

		h10box=wx.BoxSizer(wx.HORIZONTAL)
		self.StaticText_End_C=wx.StaticText(self.panel2,-1,"End Condition :")
		h10box.Add(self.StaticText_End_C,1)
		self.StaticText_End_C1=wx.StaticText(self.panel2,-1,label='_')
		h10box.Add(self.StaticText_End_C1,1)

		vbox3.Add(h10box,1,wx.EXPAND)

		h5box=wx.BoxSizer(wx.HORIZONTAL)
		self.StaticText_Inv_C=wx.StaticText(self.panel2,-1,"Invarient Condition :")
		h5box.Add(self.StaticText_Inv_C,1)
		self.StaticText_Inv_C1=wx.StaticText(self.panel2,-1,label='_')
		h5box.Add(self.StaticText_Inv_C1,1)

		vbox2.Add(h5box,1,wx.EXPAND)

		h11box=wx.BoxSizer(wx.HORIZONTAL)
		self.StaticText_Exit_C=wx.StaticText(self.panel2,-1,"Exit Condition :")
		h11box.Add(self.StaticText_Exit_C,1)
		self.StaticText_Exit_C1=wx.StaticText(self.panel2,-1,label='_')
		h11box.Add(self.StaticText_Exit_C1,1)

		vbox3.Add(h11box,1,wx.EXPAND)

		h6box=wx.BoxSizer(wx.HORIZONTAL)
		self.StaticText_Post_C=wx.StaticText(self.panel2,-1,"Post Condition :")
		h6box.Add(self.StaticText_Post_C,1)
		self.StaticText_Post_C1=wx.StaticText(self.panel2,-1,label='_')
		h6box.Add(self.StaticText_Post_C1,1)

		vbox2.Add(h6box,1,wx.EXPAND)

		h12box=wx.BoxSizer(wx.HORIZONTAL)
		self.StaticText_Skip_C=wx.StaticText(self.panel2,-1,"Skip Condition :")
		h12box.Add(self.StaticText_Skip_C,1)
		self.StaticText_Skip_C1=wx.StaticText(self.panel2,-1,label='_')
		h12box.Add(self.StaticText_Skip_C1,1)

		vbox3.Add(h12box,1,wx.EXPAND)

		hbox1=wx.BoxSizer(wx.HORIZONTAL)
		hbox1.Add(vbox2,1,wx.EXPAND)
		hbox1.Add(vbox3,1,wx.EXPAND)

		vbox4=wx.BoxSizer(wx.VERTICAL)
		vbox4.Add(hbox1,3,wx.EXPAND)
		self.Button=wx.Button(self.panel2,-1,"Add")

		self.panel2.Bind(wx.EVT_BUTTON,self.OnClicked)
		
		vbox4.Add(self.Button,0.75,wx.ALIGN_CENTRE)        
		vbox.Add(self.tree,1,wx.EXPAND)
		hbox.Add(self.panel1,1,wx.EXPAND)
		hbox.Add(self.panel2,1,wx.EXPAND)
		self.panel1.SetSizer(vbox)
		self.panel2.SetSizer(vbox4)
		self.SetSizer(hbox)
		self.Centre()


	def OnActivated(self,e):
		
		
		obj=self.tree.GetItemText(e.GetItem()).split(':')
		self.StaticText_Node_Type_List1.SetLabel(obj[0])
		self.StaticText_Node_Name1.SetLabel(obj[1])
		self.StaticText_Skip_C1.SetLabel(' ')
		self.StaticText_Post_C1.SetLabel(' ')
		self.StaticText_Exit_C1.SetLabel(' ')
		self.StaticText_Node_Parent1.SetLabel(' ')
		self.StaticText_Inv_C1.SetLabel(' ')
		self.StaticText_Pre_C1.SetLabel(' ')
		self.StaticText_End_C1.SetLabel(' ')
		self.StaticText_Repeat_C1.SetLabel(' ')
		self.StaticText_Start_C1.SetLabel(' ')
		self.StaticText_Node_Function1.SetLabel(' ')
		###	Need to get the other variables from the DB #################


	def OnClicked(self,e):

		self.frame=wx.Frame(None,-1,'Add Window',size=(900,900))

		#Box Sizer for the Node Specification part
		vbox=wx.BoxSizer(wx.VERTICAL)
		panel=wx.Panel(self.frame)
		
		#List of type of nodes
		self.List=['List','Assignment','Command','Update','Library_Call','Empty']

		#StaticBoxSizer for Node Specification
		self.StaticBox1=wx.StaticBox(panel,-1,'Node Specifications :',size=(650,100))
		self.Static_BoxSizer1=wx.StaticBoxSizer(self.StaticBox1,wx.VERTICAL)
	
		#Definitions of the text widgets in Node Specification
		self.StaticText_Node_Type_List=wx.StaticText(panel,-1,"Node Type :")
		self.Node_Type_List=wx.ComboBox(panel,-1,size=(200,-1),choices=self.List,style=wx.CB_READONLY)
		self.StaticText_Node_Parent=wx.StaticText(panel,-1,"Node Parent :")
		self.Node_Parent=wx.TextCtrl(panel,size=(200,-1))
		self.StaticText_Node_Name=wx.StaticText(panel,-1,"Node Name :")
		self.Node_Name=wx.TextCtrl(panel,size=(200,-1))
		self.StaticText_Node_Function=wx.StaticText(panel,-1,'Node Function:')
		self.Node_Function=wx.TextCtrl(panel,size=(200,50),style=wx.TE_MULTILINE)


		#Box Sizers for each Line 
		h1box=wx.BoxSizer(wx.HORIZONTAL)
		h2box=wx.BoxSizer(wx.HORIZONTAL)
		h3box=wx.BoxSizer(wx.HORIZONTAL)


		h1box.Add(self.StaticText_Node_Type_List,1)		
		h1box.Add(self.Node_Type_List,1)
		h1box.Add(self.StaticText_Node_Parent,1)
		h1box.Add(self.Node_Parent,1)
		vbox.Add(h1box,1)

		
		h2box.Add(self.StaticText_Node_Name,1)
		h2box.Add(self.Node_Name,1)
		vbox.Add(h2box,1)

	
		h3box.Add(self.StaticText_Node_Function,1)
		h3box.Add(self.Node_Function,1)
		vbox.Add(h3box,1)
		
		
		self.Static_BoxSizer1.Add(vbox,1,wx.ALL|wx.CENTER,5)

#####################End of the Node Specification #######################
##########################################################################
#####################Node Condition Part##################################


		self.Static_Box2=wx.StaticBox(panel,-1,'Node Conditions :',size=(650,100))
		self.Static_BoxSizer2=wx.StaticBoxSizer(self.Static_Box2,wx.VERTICAL)


		
		vt1box=wx.BoxSizer(wx.VERTICAL)
		vt3box=wx.BoxSizer(wx.VERTICAL)
		

		#1st Row of Node Conditions

		ht2box=wx.BoxSizer(wx.HORIZONTAL)
		ht4box=wx.BoxSizer(wx.HORIZONTAL)

		self.StaticText_Start_C=wx.StaticText(panel,-1,"Start Condition:")
		ht2box.Add(self.StaticText_Start_C,1)
		
		self.Text_Start_C=wx.TextCtrl(panel,size=(200,50),style=wx.TE_MULTILINE)
		ht2box.Add(self.Text_Start_C,1)
		
		vt1box.Add(ht2box,1,wx.EXPAND)

		self.StaticText_Pre_C=wx.StaticText(panel,-1,"Pre Condition:")
		ht4box.Add(self.StaticText_Pre_C,1)

		self.Text_Pre_C=wx.TextCtrl(panel,size=(200,50),style=wx.TE_MULTILINE)
		ht4box.Add(self.Text_Pre_C,1)

		vt3box.Add(ht4box,1,wx.EXPAND)
		
		#2nd Row of Node Conditions

		ht5box=wx.BoxSizer(wx.HORIZONTAL)
		ht6box=wx.BoxSizer(wx.HORIZONTAL)

		self.StaticText_Repeat_C=wx.StaticText(panel,-1,"Repeat Condition :")
		ht5box.Add(self.StaticText_Repeat_C,1)
		
		self.Text_Repeat_C=wx.TextCtrl(panel,size=(200,50),style=wx.TE_MULTILINE)
		ht5box.Add(self.Text_Repeat_C,1)
		
		vt1box.Add(ht5box,1,wx.EXPAND)

		self.StaticText_End_C=wx.StaticText(panel,-1,"End Condition :")
		ht6box.Add(self.StaticText_End_C,1)

		self.Text_End_C=wx.TextCtrl(panel,size=(200,50),style=wx.TE_MULTILINE)
		ht6box.Add(self.Text_End_C,1)

		vt3box.Add(ht6box,1,wx.EXPAND)

		#3rd Row of Node Conditions
					
		ht7box=wx.BoxSizer(wx.HORIZONTAL)
		ht8box=wx.BoxSizer(wx.HORIZONTAL)

		self.StaticText_Inv_C=wx.StaticText(panel,-1,"Invarient Condition :")
		ht7box.Add(self.StaticText_Inv_C,1)
		
		self.Text_Inv_C=wx.TextCtrl(panel,size=(200,50),style=wx.TE_MULTILINE)
		ht7box.Add(self.Text_Inv_C,1)
		
		vt1box.Add(ht7box,1,wx.EXPAND)
		
		self.StaticText_Exit_C=wx.StaticText(panel,-1,"Exit Condition :")
		ht8box.Add(self.StaticText_Exit_C,1)

		self.Text_Exit_C=wx.TextCtrl(panel,size=(200,50),style=wx.TE_MULTILINE)
		ht8box.Add(self.Text_Exit_C,1)

		vt3box.Add(ht8box,1,wx.EXPAND)

		#4th Row of Node Conditions

		ht9box=wx.BoxSizer(wx.HORIZONTAL)
		ht10box=wx.BoxSizer(wx.HORIZONTAL)

		self.StaticText_Post_C=wx.StaticText(panel,-1,"Post Condition :")
		ht9box.Add(self.StaticText_Post_C,1)
		
		self.Text_Post_C=wx.TextCtrl(panel,size=(200,50),style=wx.TE_MULTILINE)
		ht9box.Add(self.Text_Post_C,1)
		
		vt1box.Add(ht9box,1,wx.EXPAND)

		self.StaticText_Skip_C=wx.StaticText(panel,-1,"Skip Condition :")
		ht10box.Add(self.StaticText_Skip_C,1)

		self.Text_Skip_C=wx.TextCtrl(panel,size=(200,50),style=wx.TE_MULTILINE)
		ht10box.Add(self.Text_Skip_C,1)

		vt3box.Add(ht10box,1,wx.EXPAND)
		
##################### End of Node Conditions ##########################		
		

		vokbox=wx.BoxSizer(wx.VERTICAL)
		Ok_Button=wx.Button(panel,-1,"OK")
		vokbox.Add(Ok_Button,0,wx.ALIGN_CENTER)
		
		
		#Box Sizer for enclosing both vertical sub regions in Node Conditions
		hubox=wx.BoxSizer(wx.HORIZONTAL)
		hubox.Add(vt1box,1,wx.EXPAND)
		hubox.Add(vt3box,1,wx.EXPAND)
		
		vubox=wx.BoxSizer(wx.VERTICAL)
		vubox.Add(hubox,1,wx.EXPAND)

		self.Static_BoxSizer2.Add(vubox,1,wx.CENTER|wx.ALL,5)

		#Box Sizer for the Whole Dialog
		uvbox=wx.BoxSizer(wx.VERTICAL)
		uvbox.Add(self.Static_BoxSizer1,1.5,wx.EXPAND)
		uvbox.Add(self.Static_BoxSizer2,2,wx.EXPAND)
		uvbox.Add(vokbox,0.5,wx.EXPAND)

		panel.Bind(wx.EVT_BUTTON,self.On_Clicked)
		panel.SetSizer(uvbox)
		self.frame.Centre()	
		self.frame.Show()


	def On_Clicked(self,e):
		
		self.frame.Close()
		Node_Type=self.List[self.Node_Type_List.GetSelection()]
		Node_Parent=self.Node_Parent.GetValue()
		Node_Name=self.Node_Name.GetValue()
		Node_Function=self.Node_Function.GetValue()
		Start_C=self.Text_Start_C.GetValue()
		Pre_C=self.Text_Pre_C.GetValue()
		Repeat_C=self.Text_Repeat_C.GetValue()
		End_C=self.Text_End_C.GetValue()
		Inv_C=self.Text_Inv_C.GetValue()
		Exit_C=self.Text_Exit_C.GetValue()
		Post_C=self.Text_Post_C.GetValue()
		Skip_C=self.Text_Skip_C.GetValue()
		'''
			Need to save these variables in the database  
		'''
		if Node_Type=='List':
			self.a[str(Node_Name)]=self.tree.AppendItem(self.a[Node_Parent],Node_Type+':'+str(Node_Name))

		else:
			self.tree.AppendItem(self.a[Node_Parent],Node_Type+':'+str(Node_Name))



		


if __name__=="__main__":
	app=wx.App()
	f=Frame(None,-1,'Structure')
	f.Show()
	app.MainLoop()