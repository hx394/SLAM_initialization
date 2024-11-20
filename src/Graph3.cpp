#include "Graph.h"


Quat reverseSign(Quat q1){
    return Quat(-1*q1.w(), -1*q1.x(), -1*q1.y(), -1*q1.z());
}


bool areQuaternionsClose(Quat q1, Quat q2){
    double dot = q1.w()*q2.w() + q1.x()*q2.x() + q1.y()*q2.y() + q1.z()*q2.z();
    if(dot < 0.0) return false;
    else return true;
}


Quat averageQuaternions(std::list<Quat> quats){
    int addAmount = quats.size();

    Quat first_rotation = quats.front();
    quats.pop_front();

    double w = 1.0 / addAmount * first_rotation.w();
    double x = 1.0 / addAmount * first_rotation.x();
    double y = 1.0 / addAmount * first_rotation.y();
    double z = 1.0 / addAmount * first_rotation.z();

    for(auto it = quats.begin(); it != quats.end(); it++){
        Quat current = quats.front();
        if(!areQuaternionsClose(first_rotation, current)){
            current = reverseSign(current);
        }
        w += current.w() * 1.0 / addAmount;
        x += current.x() * 1.0 / addAmount;
        y += current.y() * 1.0 / addAmount;
        z += current.z() * 1.0 / addAmount;
    }
    Quat final_q(w,x,y,z);
    final_q.normalize();
    return final_q;
}


Graph::Graph(){ }
Graph::~Graph(){ }


void Graph::addEdge2D(Edge2D e){
    structure2D[e.getToID()].push_back(e);
    structure2D[e.getFromID()].push_back(e.getBackwards());
    edges2D.push_back(e);
}


void Graph::addEdge3D(Edge3D e){
    structure3D[e.getToID()].push_back(e);
    structure3D[e.getFromID()].push_back(e.getBackwards());
    edges3D.push_back(e);
}


void Graph::readFile(std::string inputPath){
    EdgeNum = 0;

    std::ifstream file;
    file.open(inputPath);
    if (!file) throw std::invalid_argument("ERROR: input file not found.");

    std::string line;
    getline(file,line);
    if(line.find("EDGE_SE2")==0 || line.find("VERTEX_SE2")==0) D=2;
    else if(line.find("EDGE_SE3:QUAT")==0 || line.find("VERTEX_SE3:QUAT")==0) D=3;
    else throw std::invalid_argument("ERROR: input format is not valid.");

    file.close();

    if(D==2) readFile2D(inputPath);
    else if(D==3) readFile3D(inputPath);
}


void Graph::readFile2D(std::string inputPath){
    std::ifstream file;
    file.open(inputPath);
    std::string line;
    while(getline(file,line)){
        if(line.find("EDGE_SE2") == 0)
        {
            std::istringstream iss(line);
            std::string tp;
            int from, to;
            double x,y,th,xx,xy,xth,yy,yth,thth;
            iss>>tp>>from>>to>>x>>y>>th>>xx>>xy>>xth>>yy>>yth>>thth;
            Matrix I(3,3);
            I << xx,  xy,  xth,
                 xy,  yy,  yth,
                 xth, yth, thth;

            addEdge2D(Edge2D(from, to, SE2(x, y, th), I));
            EdgeNum++;
        }
    }
    file.close();
    std::cout<<"Input loaded."<<std::endl;
    std::cout<<"Number of measurements: "<<EdgeNum<<std::endl;
}


void Graph::readFile3D(std::string inputPath){
    std::ifstream file;
    file.open(inputPath);

    std::string line;
    while(getline(file,line)){
        if(line.find("EDGE_SE3:QUAT")==0){
            std::istringstream iss(line);
            std::string tp;
            int from, to;
            double v_x, v_y, v_z, q_x, q_y, q_z, q_w;
            double i00, i01, i02, i03, i04, i05;
            double i11, i12, i13, i14, i15;
            double i22, i23, i24, i25;
            double i33, i34, i35;
            double i44, i45;
            double i55;

            iss>>tp>>from>>to>>v_x>>v_y>>v_z>>q_x>>q_y>>q_z>>q_w>>
            i00>>i01>>i02>>i03>>i04>>i05>>
            i11>>i12>>i13>>i14>>i15>>
            i22>>i23>>i24>>i25>>
            i33>>i34>>i35>>
            i44>>i45>>
            i55;

            Matrix I(6, 6);
            I << i00, i01, i02, i03, i04, i05,
                 i01, i11, i12, i13, i14, i15,
                 i02, i12, i22, i23, i24, i25,
                 i03, i13, i23, i33, i34, i35,
                 i04, i14, i24, i34, i44, i45,
                 i05, i15, i25, i35, i45, i55;

            addEdge3D(Edge3D(from, to, SE3(v_x, v_y, v_z, q_w, q_x, q_y, q_z), I));
            EdgeNum++;
        }
    }
    file.close();
    std::cout<<"Input loaded."<<std::endl;
    std::cout<<"Number of measurements: "<<EdgeNum<<std::endl;
}


void Graph::writeFile(std::string outputPath){
    if(D == 2) writeFile2D(outputPath);
    else if(D == 3)    writeFile3D(outputPath);
    std::cout<<"Output finished."<<std::endl;
}


void Graph::writeFile2D(std::string outputPath){
    std::ofstream file;
    file.open(outputPath);
    if (!file) throw std::invalid_argument("ERROR: incorrect output path.");

    for(auto x : positions2Dav){
        Node n = x.first;
        SE2 p = x.second;
        file<<"VERTEX_SE2 "<<n.getID()<<" "<<p.toString()<<std::endl;
    }
    for(auto e : edges2D){
        Matrix I = e.getI();
        file<<"EDGE_SE2 "<<e.getFromID()<<" "<<e.getToID()<<" "<<
        e.getM().toString()<<" "<<
        I(0,0)<<" "<<I(0,1)<<" "<<I(0,2)<<" "<<
                     I(1,1)<<" "<<I(1,2)<<" "<<
                                  I(2,2)<< std::endl;
    }
    file.close();
}


void Graph::writeFile3D(std::string outputPath){
    std::ofstream file;
    file.open(outputPath);
    if (!file) throw std::invalid_argument("ERROR: incorrect output path.");

    for(auto x : positions3Dav){
        Node n = x.first;
        SE3 p = x.second;
        file<<"VERTEX_SE3:QUAT "<<n.getID()<<" "<<p.toString()<<std::endl;
    }
    for(auto e : edges3D){
        Matrix I = e.getI();
        file<<"EDGE_SE3:QUAT "<<e.getFromID()<<" "<<e.getToID()<<" "<<
        e.getM().toString()<<" "<<
        I(0,0)<<" "<<I(0,1)<<" "<<I(0,2)<<" "<<I(0,3)<<" "<<I(0,4)<<" "<<I(0,5)<<" "<<
                     I(1,1)<<" "<<I(1,2)<<" "<<I(1,3)<<" "<<I(1,4)<<" "<<I(1,5)<<" "<<
                                  I(2,2)<<" "<<I(2,3)<<" "<<I(2,4)<<" "<<I(2,5)<<" "<<
                                               I(3,3)<<" "<<I(3,4)<<" "<<I(3,5)<<" "<<
                                                            I(4,4)<<" "<<I(4,5)<<" "<<
                                                                         I(5,5)<<std::endl;
    }
    file.close();
}


void Graph::runMASAT(){
    if(D==2){ rollingBFS2D();simpleAverage2D();}
    else if(D==3) {rollingBFS3D();simpleAverage3D();}
    std::cout<<"MASAT rolling BFS done."<<std::endl;
}


void Graph::rollingBFS2D(){

    
    std::map<Node, bool> marked;
    std::map<Node, bool> fixed;
    for(auto p : structure2D){
        marked.insert(std::pair<Node,bool>(p.first, false));
        fixed.insert(std::pair<Node,bool>(p.first, false));
    }
    auto fst = structure2D.begin();
    Node source = fst->first;
    std::cout<<"Running MASAT rolling BFS from VERTEX "<<source.getID()<<"."<<std::endl;

    positions2D[source] = SE2(0,0,0);
    marked[source] = true;
    fixed[source] = true;

  
	
    int layerLimit=1;
    while(true){
    	int layer=0;
        std::queue<Node> Q;
        Q=std::queue<Node>();
    	for(auto p : structure2D){
	    marked[p.first]= false;
        
		
	}
	Q.push(source);
	marked[source] = true;
	
	while(!Q.empty()){
		int levelSize=Q.size();
		for(int i=0;i<levelSize;i++){
			Node toBeFixed = Q.front();
			double s=0;
			double sx=0;
			double sy=0;
			double scos=0;
			double ssin=0;
			for(auto e : structure2D[toBeFixed]){
			    Node Neig(e.getFromID());
			    if(!marked.at(Neig)){
				Q.push(Neig);
				marked[Neig] = true;
			    }
			    if(fixed.at(Neig)){
				s+=1;
				SE2 guess = positions2D[Neig] * e.getM();
				sx += guess[0];
				sy += guess[1];
				scos += cos(guess[2]);
				ssin += sin(guess[2]);
			    }
			    
			}
			if(s!=0){
			    SE2 newp = SE2(sx/s, sy/s, atan2(ssin,scos));
			    positions2D[toBeFixed] = newp;
            }
            fixed[toBeFixed] = true;
			Q.pop();
		}
		layer=layer+1;
		if(layer==layerLimit){break;}
	}
	layerLimit=layerLimit+1;
	if(Q.empty()){
		break;
	}
    }
}


void Graph::rollingBFS3D(){

    std::map<Node, bool> marked;
    std::map<Node, bool> fixed;
    for(auto p : structure3D){
        marked.insert(std::pair<Node,bool>(p.first, false));
        fixed.insert(std::pair<Node,bool>(p.first, false));
    }
    
    auto fst = structure3D.begin();
    Node source = fst->first;
    std::cout<<"Running MASAT rolling BFS from VERTEX "<<source.getID()<<"."<<std::endl;

    positions3D[source] = SE3(0,0,0,1,0,0,0);
    marked[source] = true;
    fixed[source] = true;
    	
    int layerLimit=1;
    while(true){
    	int layer=0;	
        std::queue<Node> Q;
        Q=std::queue<Node>();
    	for(auto p : structure3D){
	    marked[p.first]= false;
	}
	Q.push(source);
	marked[source] = true;

	while(!Q.empty()){
		int levelSize=Q.size();
		for(int i=0;i<levelSize;i++){
			Node toBeFixed = Q.front();
			double s=0;
        		double v_x=0;
        		double v_y=0;
        		double v_z=0;
        		std::list<Quat> guesses;
 			
			for(auto e : structure3D[toBeFixed]){
			   Node Neig = e.getFromID();
			   if(!marked[Neig]){
				Q.push(Neig);
				marked[Neig] = true;
			   }
			   if(fixed[Neig]){
				 s+=1;
				SE3 guess = positions3D[Neig]*e.getM();
				guesses.push_back(Quat(guess[6], guess[3], guess[4], guess[5]));
				v_x += guess[0];
				v_y += guess[1];
				v_z += guess[2];
					    }
			    
			}
			
			if(s!=0){
			    Vec v_avg(v_x/s, v_y/s, v_z/s);
			    Quat q_avg = averageQuaternions(guesses);
                positions3D[toBeFixed] = SE3(v_avg, q_avg);
            }
			

			
			fixed[toBeFixed] = true;
			Q.pop();
    
			
			
		}
		layer=layer+1;
		if(layer==layerLimit){break;}
	   }
	layerLimit=layerLimit+1;
	if(Q.empty()){
		break;
	}
    }
}

void Graph::simpleAverage2D(){
    
    std::map<Node, bool> marked;
    
    for(auto p : structure2D){
        marked[p.first]= false;
        
    }
    auto fst = structure2D.begin();
    Node source = fst->first;
    std::cout<<"Running MASAT simple average from VERTEX "<<source.getID()<<"."<<std::endl;

    std::queue<Node> Q;
    Q.push(source);
    marked[source]=true;
    while(!Q.empty()){
        Node current = Q.front();
        
        
        double s=0;
        double sx=0;
        double sy=0;
        double scos=0;
        double ssin=0;
        for(auto e : structure2D[current]){
            Node Neig(e.getFromID());
            if(!marked.at(Neig)){
                Q.push(Neig);
                marked[Neig] = true;
            }
            
            s+=1;
            SE2 guess = positions2D[Neig] * e.getM();
            sx += guess[0];
            sy += guess[1];
            scos += cos(guess[2]);
            ssin += sin(guess[2]);
            
        }
        SE2 newp = SE2(sx/s, sy/s, atan2(ssin,scos));
        positions2Dav[current] = newp;
        Q.pop();
    }
}


void Graph::simpleAverage3D(){
    std::map<Node, bool> marked;
    
    for(auto p : structure3D){
        marked[p.first]= false;
        
    }
    auto fst = structure3D.begin();
    Node source = fst->first;
    std::cout<<"Running MASAT simple average from VERTEX "<<source.getID()<<"."<<std::endl;

    
    marked[source] = true;
    

    std::queue<Node> Q;
    Q.push(source);

    while(!Q.empty()){
        Node current = Q.front();

        double s=0;
        double v_x=0;
        double v_y=0;
        double v_z=0;
        std::list<Quat> guesses;

        for(auto e : structure3D[current]){
            Node Neig = e.getFromID();

            if(!marked[Neig]){
                Q.push(Neig);
                marked[Neig] = true;
            }
          
            s+=1;
            SE3 guess = positions3D[Neig]*e.getM();
            guesses.push_back(Quat(guess[6], guess[3], guess[4],guess[5]));
            v_x += guess[0];
            v_y += guess[1];
            v_z += guess[2];
            

        }
        Vec v_avg(v_x/s, v_y/s, v_z/s);
        Quat q_avg = averageQuaternions(guesses);
        positions3Dav[current] = SE3(v_avg, q_avg);
        Q.pop();
    }
}
