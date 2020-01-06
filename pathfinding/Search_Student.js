// Search_Student.js 
// Computer Science 3200 - Assignment 2
// Author(s): Yimiao Zhang 201803269 Zuchi Zheng 201503869

class Search_Student {

    constructor(grid, config) {
        this.config = config;       // search configuration object
                                    //   config.actions = array of legal [x, y] actions
                                    //   config.actionCosts[i] = cost of config.actions[i]
                                    //   config.heuristic = 'diag', 'card', 'dist', or 'zero'
        this.name = "Student";
        this.grid = grid;           // the grid we are using to search
        this.sx = -1;               // x location of the start state
        this.sy = -1;               // y location of the start state
        this.gx = -1;               // x location of the goal state
        this.gy = -1;               // y location of the goal state
        this.size = 1;              // the square side length (size) of the agent
        this.maxSize = 3;           // the maximum size of an agent

        this.inProgress = false;    // whether the search is in progress
        this.expanded = 0;          // number of nodes expanded (drawn in GUI)

        this.path = [];             // the path, if the search found one
        this.open = [];             // the current open list of the search (stores Nodes)
        this.closed = [];           // the current closed list of the search
        this.cost = 'Search Not Completed'; // the cost of the path found, -1 if no path
        this.objectSize= 1;
        this.computeSectors();
    }
    
    //set up all the necessary data structures to begin a new search
    startSearch(sx, sy, gx, gy, size) {
        // deals with an edge-case with the GUI, leave this line here
        if (sx == -1 || gx == -1) { return; }

        this.inProgress = true;     // the search is now considered started
        this.sx = sx;               // set the x,y location of the start state
        this.sy = sy;
        this.gx = gx;               // set the x,y location of the goal state
        this.gy = gy;
        this.size = size;           // the size of the agent
        this.path = [];             // set an empty path
        this.open = [];
        this.closed = [];
        this.objectSize = size;
        let h = this.estimateCost(sx, sy, gx, gy);
        let firstNode = new Node(sx, sy, null, [0, 0], 0, h);
        
        this.addOpen(firstNode);
    
    }

    //compute and return the heuristic function h(n) of a given start location to a given goal location.
    estimateCost(x, y, gx, gy) {  
        let abx = Math.abs(gx - x);
        let aby = Math.abs(gy - y);
        // compute and return the diagonal manhattan distance heuristic
        if (this.config.heuristic == 'diag') {
            return (abx + aby) * 100 - Math.min(abx, aby) * (200 - 141);  
        // compute and return the 4 directional (cardinal) manhattan distance
        } else if (this.config.heuristic == 'card') {
            return (abx + aby) * 100;
        // compute and return the 2D euclidian distance (Pythagorus)
        } else if (this.config.heuristic == 'dist') {
            let distx = abx * abx;
            let disty = aby * aby;
            return (Math.sqrt(distx+disty)) * 100;
        // return zero heuristic
        } else if (this.config.heuristic == 'zero') {
            return 0;
        }
    }



    //return whether or not the two given locations are connected
    isConnected(x1, y1, x2, y2, size) {
        if ((this.sectors[size][x1][y1] == this.sectors[size][x2][y2]) && (this.sectors[size][x1][y1] != 0)){
            return true;
        }
        return false;   
    }

    //return whether or not the given action is able to be performed from the given (x,y) location.
    isLegalAction(x, y, size, action) {
        let nx = x + action[0];
        let ny = y + action[1];
        if((this.grid.isOOB(x,y,size)) || this.grid.isOOB(nx, ny, size)){
            return false;
        }
        
        if( this.grid.get(x,y) != this.grid.get(nx, ny) ){
            return false;
        }
        if(this.isConnected(x,y,nx,ny,size) && this.isConnected(x,y,x,ny,size) && this.isConnected(x,y,nx,y,size)){
            return true;
        }
        return false;
    }


    //compute and store the connected sectors
    computeSectors() {
        //initialize empty sectors array
        this.sectors = [];
        for (let s = 0; s <= this.maxSize; s++) {
            let temp = [];
            for (let w = 0; w < this.grid.width; w++) {
                temp[w] = [];
                for (let h = 0; h < this.grid.height; h++) {
                    temp[w][h] = 0;
                }
            }
            this.sectors[s] = temp;
        }
        
        //set up sectorNumbers array 
        this.sectorNumber = [1, 1, 1, 1];

        for (let s = 1; s <= this.maxSize; s++) {
            for (let w = 0; w < this.grid.width; w++) {
                for (let h = 0; h < this.grid.height; h++) {

                    let sectorOpen = [[w, h]];
                    let gridNum = this.grid.get(w,h);
                    // for loop go through sector's every single location
                    for (let i = 0; i < sectorOpen.length; i++) {
                        //continue if the grid is out of boundary
                        if (this.grid.isOOB(sectorOpen[i][0], sectorOpen[i][1], s)) {
                            continue;
                        }
                        //check if the grid values are the same
                        if (gridNum != this.grid.get(sectorOpen[i][0], sectorOpen[i][1])){
                            continue;
                        }
                        //stop if it has been assigned value before
                        if (this.getSector(s, sectorOpen[i][0], sectorOpen[i][1]) != 0) {
                            continue;
                        }
                        //check if the current location can fit 
                        if (!this.canFit(sectorOpen[i][0], sectorOpen[i][1], s)) {
                            continue;
                        }
                        //assign sectorNumber to the sector 
                        this.sectors[s][sectorOpen[i][0]][sectorOpen[i][1]] = this.sectorNumber[s];

                        //4 cardinal actions
                        let allActions = [[1,0], [-1,0],[0,1],[0,-1]];
                        for(let j = 0; j < allActions.length; j++){
                            sectorOpen.push([sectorOpen[i][0] + allActions[j][0], sectorOpen[i][1] + allActions[j][1]]);

                        }   

                    }
                //increase the sector number if the sector number has been used to assign to location 
                    if(!(sectorOpen.length <=1)){
                        this.sectorNumber[s]++;
                    }
                }
            }
    
        }
    }
    //return sectors location
    getSector(size, x, y) {
        return this.sectors[size][x][y];
    }
    
    canFit(x, y, size) {
        let gridNum = this.grid.get(x, y);
        let s = size;
        //horizontal
        for(let i = 0; i < s; i++){
            //vertical
            for(let j = 0; j < s; j++)
                if ( gridNum != this.grid.get(x+i, y+j)){
            return false;
            }
        }
        return true;
    }

    
    //performs one iteration of search
    searchIteration() {
        
        // if we've already finished the search, do nothing
        if (!this.inProgress) { return; }

        //if open list is empty, do nothing
        if(this.openSize() <= 0){
            this.inProgress = false;
            this.path = [];
            this.cost = -1;
            return;
        }
        //check if the start and end goals are connected
        if (!this.isConnected(this.sx, this.sy, this.gx, this.gy, this.objectSize)) { 
            this.inProgress = false; 
            this.cost = -1; 
            return; 
        }

        //remove the node from the open list
        let minIndex = this.getMinNodeIndex(this.open);
        //let currentNode = new Node();
        let currentNode = this.open[minIndex];
        this.open.splice(minIndex,1);

        //if found the goal, return path
        if(this.findGoal(currentNode.x, currentNode.y)){
            this.path = this.buildPath(currentNode);
            this.cost = currentNode.g;
            this.inProgress = false;          
            return;
        }

        //check if the node is on the closed list
        if(this.checkClosed(currentNode)){
            return;
        }
        //add the node to the closed list
        this.addClosed(currentNode);

        let currentx = currentNode.x;
        let currenty = currentNode.y;
        let currentg = currentNode.g;

        
        for (var i = 0; i < this.config.actions.length; i++) {
            if (this.isLegalAction(currentx, currenty, this.objectSize, this.config.actions[i])) {
                let nx = currentx + this.config.actions[i][0];
                let ny = currenty + this.config.actions[i][1];
                let ng = currentg + this.config.actionCosts[i];
                let nh = this.estimateCost(nx, ny, this.gx, this.gy);
                let newNode = new Node(nx, ny, currentNode, this.config.actions[i], ng, nh);      

                this.addOpen(newNode);
                
            }
        }

    }

    //returns the open list states
    getOpen() {
        let tempList = [];
        for (let i = 0; i < this.open.length; i++) {
            tempList.push([this.open[i].x, this.open[i].y]);
        }
        return tempList;
    }
    //add the node to the open list
    addOpen(n){
        this.open.push(n);
    }
    //return the length of the open list
    openSize(){
        return this.open.length;
    }

    //check if the node is on the open list
    checkOpen(node){
        for (let i = 0; i< this.open.length; i++){
            if ( node.x == this.open[i].x && node.y == this.open[i].y){
                return true;
            }
        } 
        return false;
    }

    // return the closed list states
    getClosed() {
        let tempList = [];
        for (let i =0; i < this.closed.length; i++){
            tempList.push([this.closed[i].x,this.closed[i].y]);
        }
        return tempList;
    }
    //add the node to the closed list
    addClosed(node){
        this.closed.push(node);
    }
    //check if the node is in the closed list
    checkClosed(node){
        for (let i = 0; i< this.closed.length; i++){
            if (node.x == this.closed[i].x && node.y == this.closed[i].y){
                return true;
            }
        } 
        return false;
    }
    //return the index of node with the smallest heuristic function value
    getMinNodeIndex(list){
        let minIndex = 0;
        let minNum = list[0].g + list[0].h;
        for(let i = 0; i < list.length; i++){
            if ( (list[i].g + list[i].h) < minNum){
                minNum = list[i].g + list[i].h;
                minIndex = i;
            }
        }
        return minIndex;    
    }
    //check if the node is the goal 
    findGoal(nodex, nodey){
        if((nodex == this.gx) && (nodey == this.gy)){
            return true;
        }
        return false;
    }    
    //return a list of action
    buildPath(p){
        let list = [];
        while(p.parent != null){
            list.push(p.action);
            p = p.parent;
        }
        list.reverse();
        return list;
    }

}


class Node {
    constructor(x, y, parent, action, g, h) {
        this.x = x;
        this.y = y;
        this.action = action;
        this.parent = parent;
        this.g = g;
        this.h = h;
    }
}