#include <iostream>
#include <vector>
#include <limits>
#include <random>
#include <cmath>
#include <algorithm>
#include <functional>
#include <fstream>
#include <time.h>


using namespace std;

#define INF 99999999.0	

typedef std::vector<float> vec_float;
typedef std::vector<int> vec_int;
typedef std::vector<vec_float> vec_states;

std::random_device rd;  //Will be used to obtain a seed for the random number engine
std::mt19937_64 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
std::uniform_real_distribution<> uni_dis(0,1);

const int dim_state=2;

//structure for cuboid region
struct Region
{
	std::vector<float> center;
	std::vector<float> size;
};

typedef std::vector<struct Region> vec_region;

// motion planning problem data
struct MPP
{
	Region operating_region;
	std::vector<float> xInit_state;
	std::vector<float> xGoal_state;
	std::vector<struct Region> obstacles;
	int dim_state;	
};


// Vertex storing information a node in the graph
struct Vertex
{
	int id;
	std::vector<float> state;
	float c2g_value;
	float lmc_value;
	float h_value;
	std::vector<float> key;
	int parent_id;
	std::vector<int> children;
	std::vector<int> outgoing_edges;
	std::vector<int> incoming_edges;
	//bool in_queue;

};

// Graph structure, stored as a vector of Vertices
struct RRG
{
	std::vector<struct Vertex> vertices;
	std::vector<vec_float> states;
	int xInit_id;
	float xInit_c2gvalue;
	float init_bias;
	float local_bias;
	int vertices_end;
	vec_int xInit_list;
	std::vector<int> B;
};


// Vector Functions
float Norm(vec_float vec)
{
	float norm=0;
	for(int i=0; i<vec.size(); i++)
	{
		norm=norm+(vec[i]*vec[i]);
	}

	return pow(norm,0.5);
}

vec_float subVector(vec_float v1, vec_float v2, int dim_state)
{
	vec_float vec (dim_state,0.);

	std :: transform(v1.begin(),v1.end(),v2.begin(),vec.begin(),std::minus<float>());

	return(vec);
}

vec_float addVector(vec_float v1,vec_float v2,int dim_state)
{
	vec_float vec (dim_state,0.);

	std :: transform(v1.begin(),v1.end(),v2.begin(),vec.begin(),std::plus<float>());

	return(vec);
}

vec_float mulVector(vec_float v,float scalar)
{
	for(int k=0;k<v.size();k++)
	{
		v[k]=v[k]*scalar;
	}

	return(v);
}

void printVector(vec_float vec)
{
	for(int k=0;k<vec.size();k++)
	{
		cout<<vec.at(k)<<",";
	}

	cout<<endl;
}

void printVectorInt(vec_int vec)
{
	for(int k=0;k<vec.size();k++)
	{
		cout<<vec.at(k)<<",";
	}

	cout<<endl;
}

bool equalVector(vec_float v1,vec_float v2)
{
	bool equal=true;

	for(int k=0;k<v1.size();k++)
	{
		if(v1.at(k)!=v2.at(k))
		{
			equal=false;
			return equal;
		}
	}
}


// Set Obstacles
vec_region loadObstacles(void)
{
	vec_region obstacles;

	Region ob1,ob2,ob3,ob4,ob5,ob6,ob7;

	ob1.center={0,2};
	ob1.size={8.5,1};

	obstacles.push_back(ob1);

	ob2.center={-4,8};
	ob2.size={1,2.5};

	obstacles.push_back(ob2);

	ob3.center={-4,4.5};
	ob3.size={4,2};
	obstacles.push_back(ob3);

	ob4.center={7,3.5};
	ob4.size={2.5,11};
	obstacles.push_back(ob4);

	ob5.center={3,5};
	ob5.size={2.5,2};
	obstacles.push_back(ob5);

	ob6.center={-6,-4};
	ob6.size={3,5};
	obstacles.push_back(ob6);

	ob7.center={3,-5};
	ob7.size={2.5,5};
	obstacles.push_back(ob7);

	return(obstacles);

}

/*Set the node in the workspace interval*/
vec_float adjInterval(vec_float x_state,MPP mpp_data)
{
	Region op_region=mpp_data.operating_region;

	vec_float xAdj_state=x_state;

	for(int i=0;i<mpp_data.dim_state;i++)
	{
		float v_min=op_region.center.at(i)-op_region.size.at(i)/2.;
		float v_max=op_region.center.at(i)+op_region.size.at(i)/2.;

		if(x_state.at(i)<v_min)
		{
			xAdj_state[i]=v_min;
		}			
		else if(x_state.at(i)>v_max)
		{
			xAdj_state[i]=v_max;
		}
	}

	return(xAdj_state);
}


/* Class that holds all the PI RRT# functions */
class Algorithm
{
public:
	float num_step;
	float steer_length;
	float ball_radius;
	float gamma;

	void printParams(RRG rrg_data)
	{
		cout <<"no. of steps:" << num_step <<endl ;
		cout <<"steer length::" << steer_length <<endl ;
		cout <<"ball radius:" << ball_radius <<endl ;
		cout <<"gamma:" << gamma <<endl ;

		cout << rrg_data.xInit_id;
	}

	/*function to compute cost (euclidean distance) between two nodes*/
	float computeCost(vec_float xFrom_state,vec_float xTo_state,int dim_state)
	{
		float dist = Norm(subVector(xFrom_state,xTo_state,dim_state));

		return dist;
	}


	/*Initialize the vertex to a new state*/
	Vertex Initialize(vec_float xNew_state)
	{

		Vertex xNew;
		xNew.id=0;
		xNew.state = xNew_state;
		xNew.c2g_value=INF;
		xNew.lmc_value=INF;
		xNew.h_value=0;
		xNew.key={INF,INF};

		xNew.parent_id=0;
		//xNew.in_queue=false;

		return(xNew);
	}

	/*function to add the vertex to the graph*/
	RRG addVertex(Vertex xNew, RRG rrg_data)
	{
		xNew.id=rrg_data.vertices_end;
		//add the vertex
		rrg_data.vertices.insert(rrg_data.vertices.begin()+rrg_data.vertices_end, xNew);
		// Add the state vector 
		rrg_data.states.insert(rrg_data.states.begin()+rrg_data.vertices_end, xNew.state);

		rrg_data.vertices_end=rrg_data.vertices_end+1;

		return rrg_data;		

	}

	/*Obstacle detection function exploiting cuboid structure of the obstacles*/

	// Check of the given point is on obstacle
	bool onObstacle(vec_float x_state,MPP mpp_data)
	{
		int num_obstacles=mpp_data.obstacles.size();

		bool collision;
		
		for(int k=0;k<num_obstacles;k++)
		{
			Region obstacle=mpp_data.obstacles.at(k);

			collision=true;

			for(int i=0;i<mpp_data.dim_state;i++)
			{
				float v_min=obstacle.center.at(i)-obstacle.size.at(i)/2.;
				float v_max=obstacle.center.at(i)+obstacle.size.at(i)/2.;

				collision = collision && (v_min <= x_state.at(i)) && (x_state.at(i) <= v_max);
			}

			if(collision==true)
			{
				return(collision);
			}

		}

		collision=false;

		return(collision);
	}

	// check for obstacles between given two points
	bool obstacleFree(vec_float xInit_state,vec_float xFinal_state,MPP mpp_data,float num_step)
	{
		bool no_collision=true;

		vec_float edir=subVector(xFinal_state,xInit_state,mpp_data.dim_state);

		float dist=Norm(edir);

		edir=mulVector(edir,1/dist);

		float step_size=dist/num_step;

		for(int k=1;k<num_step;k++)
		{
			vec_float x_curr=mulVector(edir,k*step_size);

			x_curr=addVector(x_curr,xInit_state,mpp_data.dim_state);

			if(Algorithm:: onObstacle(x_curr,mpp_data))
			{
				no_collision=false;

				return no_collision;
			}

		}

		return no_collision;
	}

	/* TO DO: Add FLANN for fast NN search*/
	int getNearest(vec_float x_state, RRG rrg_data,MPP mpp_data)
	{
		float dist=INF;
		int xNearest_id;

		for(int k=0;k<rrg_data.vertices_end;k++)
		{
			float dist_new=Norm(subVector(x_state,rrg_data.states.at(k),mpp_data.dim_state));

			if (dist_new<dist)
			{
				dist=dist_new;
				xNearest_id=k;
			}
		}

		return(xNearest_id);
	}

	/*Steer functtion */
	vec_float Steer(vec_float xFrom_state,vec_float xTo_state,float steer_length,int dim_state)
	{
		
		vec_float xSteered;

		vec_float edir=subVector(xFrom_state,xTo_state,dim_state);
		
		float dist=Norm(edir);

		if(dist<=steer_length)
		{
			xSteered=xFrom_state;
		}
		else
		{
			//cout <<"type 2 steer"<<endl;

			edir=mulVector(edir,1/dist);

			xSteered=mulVector(edir,steer_length);

			xSteered=addVector(xSteered,xTo_state,dim_state);

			//printVector(xSteered);

		}

		//xSteered=adjInterval(xSteered,mpp_data);

		return(xSteered);
	}

	/* Get points in a certain radius TO DO: Add FLANN*/	
	vec_int getNear(vec_float x_state,float radius,RRG rrg_data,MPP mpp_data)
	{
		vec_int near_list;

		int j=0;

		for(int k=0;k<rrg_data.vertices_end;k++)
		{
			float dist=Norm(subVector(rrg_data.states.at(k),x_state,mpp_data.dim_state));

			if(dist<radius)
			{
				near_list.push_back(k);
				j=j+1;
			}
		}

		return near_list;
	}


	/*Extend Procedure*/
	RRG Extend(vec_float x_state,MPP mpp_data,RRG rrg_data)
	{
		bool success=false;

		/*Check if init state is found */
		if(rrg_data.xInit_id==0)
		{
			float dist = Norm(subVector(x_state,mpp_data.xInit_state,mpp_data.dim_state));
			if (dist<.5)
			{	
				/* If within small distance, make current state the init state*/
				x_state=mpp_data.xInit_state;
			}
			//cout<<"dist:"<<dist<<endl;
		}

		int xNearest_id = Algorithm :: getNearest(x_state,rrg_data,mpp_data);

		vec_float xNearest_state=rrg_data.states.at(xNearest_id);

		//cout<<"xRandom_state:";
		//printVector(x_state);

		//cout<<"xNearest_state:";
		//printVector(xNearest_state);

		vec_float xNew_state=Algorithm :: Steer(x_state,xNearest_state,steer_length,mpp_data.dim_state);		

		xNew_state=adjInterval(xNew_state,mpp_data);

		//cout<<"xSteered:";
		//printVector(xNew_state);

		if (Algorithm:: obstacleFree(xNew_state,xNearest_state,mpp_data,num_step))
		{
			//cout<<"obstacleFree"<<endl;

			Vertex xNew=Algorithm :: Initialize(xNew_state);

			xNew.h_value=Algorithm :: computeCost(mpp_data.xInit_state,xNew_state,mpp_data.dim_state);

			xNew.c2g_value=rrg_data.vertices[xNearest_id].c2g_value + Algorithm:: computeCost(xNew_state,xNearest_state,mpp_data.dim_state);

			xNew.parent_id=xNearest_id;

			vec_int xNear_list=Algorithm:: getNear(xNew_state,ball_radius,rrg_data,mpp_data);

			if (xNear_list.size()==0)
			{
				xNear_list={xNearest_id};
			}

			int j=0;

			/*Initialize the current vertex to the best parent available*/

			for (int k=0;k<xNear_list.size();k++)
			{

				int xNear_id=xNear_list.at(k);

				vec_float xNear_state=rrg_data.states.at(xNear_id);

				if (Algorithm:: obstacleFree(xNew_state,xNear_state,mpp_data,num_step))
				{
					float temp_cost=Algorithm:: computeCost(xNew_state,xNear_state,mpp_data.dim_state) + rrg_data.vertices[xNear_id].c2g_value;

					if(xNew.c2g_value>temp_cost)
					{
						xNew.c2g_value=temp_cost;

						xNew.parent_id=xNear_id;
					}

					//xNew.outgoing_edges.insert(xNew.outgoing_edges.begin()+j,xNear_id);
					//j=j+1;

					xNew.outgoing_edges.push_back(xNear_id);

				}

			}

			xNew.incoming_edges=xNew.outgoing_edges;

			rrg_data=Algorithm:: addVertex(xNew,rrg_data);

			int xNew_id=rrg_data.vertices_end-1;

			rrg_data.vertices[xNew.parent_id].children.insert(rrg_data.vertices[xNew.parent_id].children.end(),xNew_id);

			if(equalVector(xNew_state,mpp_data.xInit_state))
			{
				rrg_data.xInit_id=xNew_id;
			}

			float dist = Norm(subVector(xNew_state,mpp_data.xInit_state,mpp_data.dim_state));


			if ((dist<0.5)&& (xNew.c2g_value<rrg_data.xInit_c2gvalue))
			{
				//rrg_data.xInit_id=xNew_id;
				rrg_data.xInit_c2gvalue=xNew.c2g_value;
				rrg_data.xInit_list.push_back(xNew_id);
			}	

			for(int k=0;k<xNew.outgoing_edges.size();k++)
			{
				int xHead_id=xNew.outgoing_edges.at(k);

				//rrg_data.vertices[xHead_id].incoming_edges.insert(rrg_data.vertices[xHead_id].incoming_edges.end(),xHead_id);

				//rrg_data.vertices[xHead_id].outgoing_edges.insert(rrg_data.vertices[xHead_id].outgoing_edges.end(),xHead_id);

				rrg_data.vertices[xHead_id].incoming_edges.push_back(xNew_id);

				rrg_data.vertices[xHead_id].outgoing_edges.push_back(xNew_id);
			}


			Vertex xParent=rrg_data.vertices.at(xNew.parent_id);

			float f_value = xParent.h_value + xParent.c2g_value;

			float c2g_init;

			if(rrg_data.xInit_id==0)
			{
				c2g_init=INF;
			}
			else
			{
				c2g_init=rrg_data.vertices[rrg_data.xInit_id].c2g_value;
			}

			/*promising verteices included for PI procedures*/
			if(f_value<c2g_init)
			{
				rrg_data.B.insert(rrg_data.B.end(),xNew_id);
			}


			/*ball radius calculations*/
			int n=rrg_data.vertices_end;
			float d=mpp_data.dim_state;
			float vball;

			if(d==2)
			{
				vball=3.1414;
			}
			if(d==3)
			{
				vball=(4/3.)*3.1414;
			}

			float new_ball_radius=pow((gamma*log(n)/(n*vball)),(1/d));

			if(new_ball_radius<Algorithm:: ball_radius)
			{
				Algorithm:: ball_radius=new_ball_radius;

				cout<<"Algo ball_radius:"<<Algorithm::ball_radius<<endl;
			}

		}

		//cout<<"xNew_state:"<<xNew_state[0]<<","<<xNew_state[1]<<endl;

		return rrg_data;
	}
	/*Policy evaluation module.*/
	RRG evaluatePI(MPP mpp_data,RRG rrg_data)
	{

		//cout<<"*********starting evaluatePI***************"<<endl;

		vec_int q;

		/*TO FIND: J value to init vertex*/
		float J_init;

		if(rrg_data.xInit_id==0)
		{
			J_init=INF;
		}
		else
		{
			

			int xState_id=rrg_data.xInit_id;

			while(xState_id!=0)
			{
				q.push_back(xState_id);

				xState_id=rrg_data.vertices[xState_id].parent_id;
			}

			rrg_data.vertices[0].c2g_value=0.;

			int xState_parent_id;

			while(q.size()>0)
			{
				xState_id=q.back();

				q.pop_back();

				xState_parent_id=rrg_data.vertices[xState_id].parent_id;

				rrg_data.vertices[xState_id].c2g_value = rrg_data.vertices[xState_parent_id].c2g_value + Algorithm:: computeCost(rrg_data.states.at(xState_id),rrg_data.states.at(xState_parent_id),mpp_data.dim_state);

			}

			J_init=rrg_data.vertices[rrg_data.xInit_id].c2g_value;
		}

		vec_int B_list={0};

		vec_int q1;
		q1.push_back(0);

		vec_float xState;
		int xState_id;

		int xNear_id;
		vec_float xNear_state;
		vec_int xNear_list;

		float J_xState;

		//cout<<"found J_init"<<endl;

		vec_int closed_list;
		std::vector<int>::iterator it;

		while(q1.size()>0)
		{
			xState_id=q1.front();

			q1.erase(q1.begin());

			closed_list.push_back(xState_id);

			//printVectorInt(q1);			

			cout<<"qsize:"<<q1.size()<<endl;

			xState=rrg_data.states.at(xState_id);

			printVector(xState);

			J_xState = rrg_data.vertices[xState_id].c2g_value;

			if(rrg_data.vertices[xState_id].h_value + J_xState < J_init)
			{
				xNear_list=rrg_data.vertices[xState_id].outgoing_edges;

				cout<<"xNear_listsize:"<<xNear_list.size()<<endl;

				for(int k=0;k<xNear_list.size();k++)
				{
					xNear_id=xNear_list.at(k);

					if(rrg_data.vertices[xNear_id].parent_id==xState_id)
					{
						xNear_state=rrg_data.states[xNear_id];

						rrg_data.vertices[xNear_id].c2g_value = Algorithm:: computeCost(xState,xNear_state,mpp_data.dim_state) + J_xState;

					}

					it=find(closed_list.begin(),closed_list.end(),xNear_id);

					if(it!=closed_list.end())
					{
						B_list.push_back(xNear_id);

						q1.push_back(xNear_id);

					}

					
				}
			}

		}

		rrg_data.B = B_list;

		return rrg_data;	
	}

	/*Policy improvement module*/

	RRG replanPI(MPP mpp_data,RRG rrg_data,float epsilon)
	{
		//float delta_J;		

		while(true)
		{
			cout<<"start replanning"<<endl;
			float delta_J = -INF;

			for(int k=0;k<rrg_data.B.size();k++)
			{
				int xCurr_id=rrg_data.B.at(k);

				vec_float xCurr_state=rrg_data.states.at(xCurr_id);

				float J = rrg_data.vertices[xCurr_id].c2g_value;

				int xCurr_parent_id=rrg_data.vertices[xCurr_id].parent_id;

				vec_int xSucc_list=rrg_data.vertices[xCurr_id].outgoing_edges;

				for (int i=0;i<xSucc_list.size();i++)
				{
					int xNeigh_id=xSucc_list.at(i);

					vec_float xNeigh_state=rrg_data.states.at(xNeigh_id);

					float temp_cost = Algorithm:: computeCost(xCurr_state,xNeigh_state,mpp_data.dim_state)+rrg_data.vertices[xNeigh_id].c2g_value;

					if(J>temp_cost)
					{
						J = temp_cost;

						//xCurr_parent_id=xNeigh_id;

						rrg_data.vertices[xCurr_id].parent_id=xNeigh_id;

					}
				}

				float temp_delta_J=abs(rrg_data.vertices[xCurr_id].c2g_value - J);

				if (temp_delta_J>delta_J)
				{
					delta_J=temp_delta_J;
				}

				//********************check later*************************

				//rrg_data.vertices[xCurr_id].parent_id=xCurr_parent_id;

				//rrg_data.vertices[xCurr_id].c2g_value=J;
			}

			if(delta_J<=epsilon)
			{
				return rrg_data;
			}

			else
			{
				rrg_data=Algorithm:: evaluatePI(mpp_data,rrg_data);
			}

		}
	}
	/*Random state generator*/
	vec_float generateRandomState(MPP mpp_data, RRG rrg_data)
	{
		vec_float xRandom_state (mpp_data.dim_state,0.);

		float urand = uni_dis(gen);

		//if ((rrg_data.xInit_id==0) && (urand<rrg_data.init_bias))
		if ((urand<rrg_data.init_bias))
		{
			xRandom_state=mpp_data.xInit_state;

			return(xRandom_state);
		}

		else
		{
			Region op_region = mpp_data.operating_region;

			while(true)
			{
				for(int i=0;i<mpp_data.dim_state;i++)
				{
					float x_min=op_region.center.at(i)-op_region.size.at(i)/2.;
					float x_max=op_region.center.at(i)+op_region.size.at(i)/2.;
					float urand = uni_dis(gen);
					xRandom_state[i]=x_min+(x_max-x_min)*urand;
				}

				//cout<<"Random state:"<<xRandom_state[0]<<","<<xRandom_state[1]<<endl;

				bool collision_check=Algorithm :: onObstacle(xRandom_state,mpp_data);
				
				if(!collision_check)
				{
					return(xRandom_state);
				}

			}
		}		
	}


	RRG PIRRTsharp(MPP mpp_data,RRG rrg_data, int max_iter)
	{
		Vertex xGoal;
		xGoal=Algorithm :: Initialize(mpp_data.xGoal_state);

		xGoal.lmc_value=0;
		xGoal.c2g_value=0;
		xGoal.h_value=Algorithm:: computeCost(mpp_data.xGoal_state,mpp_data.xInit_state,mpp_data.dim_state);
		//xGoal.key={0,0};

		rrg_data=Algorithm :: addVertex(xGoal,rrg_data);		

		for(int iter=0; iter<=max_iter; iter++)
		{
			cout <<"iterno:"<<iter<<endl;

			vec_float xRandom_state = Algorithm :: generateRandomState(mpp_data,rrg_data);

			cout<<"random state:";
			printVector(xRandom_state);

			int B_size=rrg_data.B.size();

			rrg_data=Algorithm :: Extend(xRandom_state,mpp_data,rrg_data);

			cout<<"ball_radius:"<<ball_radius<<endl;

			if(rrg_data.B.size()>B_size)
			{
				rrg_data = Algorithm:: replanPI(mpp_data,rrg_data,.0001);
			}

		}

		return(rrg_data);
	}

	
};


vec_states getPath(RRG rrg_data)
{
	vec_states path;

	float minc2g=INF;

	int xState_id;

	int xid;

	for(int i=0;i<rrg_data.xInit_list.size();i++)
	{
		xid=rrg_data.xInit_list.at(i);

		cout<<"c2g init values:"<<rrg_data.vertices[xid].c2g_value<<endl;

		if(rrg_data.vertices[xid].c2g_value<minc2g)
		{
			xState_id=xid;
			minc2g=rrg_data.vertices[xid].c2g_value;
		}

	}

	cout<<"min init val:"<<minc2g<<endl;	

	vec_float xState;

	while(xState_id!=0)
	{
		xState=rrg_data.states.at(xState_id);

		path.push_back(xState);

		xState_id=rrg_data.vertices[xState_id].parent_id;
	}

	path.push_back(rrg_data.states.at(0));

	return path;
}

void writeSolution(vec_states path,RRG rrg_data)
{
	ofstream myfile;
	myfile.open ("solution.txt");

	for(int k=0;k<path.size();k++)
	{
		vec_float xState=path.at(k);

		printVector(xState);

		myfile<<xState[0]<<","<<xState[1]<<endl;
	}

	myfile.close();

	myfile.open ("states.txt");

	for(int k=0;k<rrg_data.states.size();k++)
	{
		vec_float xState=rrg_data.states.at(k);

		int xParent_id=rrg_data.vertices[k].parent_id;

		vec_float xParent=rrg_data.states.at(xParent_id);

		myfile<<xState[0]<<","<<xState[1]<<","<<xParent[0]<<","<<xParent[1]<<endl;
	}

	myfile.close();
}

int main()
{
	clock_t t1,t2;
	t1=clock();

	Algorithm alg;

	alg.num_step=10;
	alg.steer_length=1;
	alg.ball_radius=1.7*alg.steer_length;
	alg.gamma=1000;

	MPP mpp_data;
	mpp_data.dim_state=dim_state;
	mpp_data.xInit_state={0,0};
	mpp_data.xGoal_state={0,8};

	mpp_data.operating_region.center={0,0};
	mpp_data.operating_region.size={20,20};
	mpp_data.obstacles=loadObstacles();	

	cout <<"no. of obstacles:"<< mpp_data.obstacles.size()<<endl;

	RRG rrg_data;
	rrg_data.xInit_id=0;
	rrg_data.vertices_end=0;
	rrg_data.init_bias=0.1;
	rrg_data.local_bias=0.1;
	rrg_data.xInit_c2gvalue=INF;	

	alg.printParams(rrg_data);

	int max_iter=1500;

	rrg_data=alg.PIRRTsharp(mpp_data,rrg_data,max_iter);

	t2=clock();

	float process_time=(float)t2-(float)t1;

	cout<<"time:"<<process_time<<endl;

	cout<<"no of vertices:"<<rrg_data.states.size()<<endl;	

	cout<<"xInit_id:"<<rrg_data.xInit_id<<endl;

	vec_states path = getPath(rrg_data);

	writeSolution(path,rrg_data);


	//truct PlanningProblem mpp_data; 

	return 0;


}






