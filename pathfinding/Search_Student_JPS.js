// Search_Student.js 
// Computer Science 3200 - Final Project 
// Author(s): Yimiao Zhang (201803269) Zuchi Zheng (201503869)

class Search_Student_JPS{

    constructor(grid, config) {
        this.config = config;       // search configuration object
                                    //   config.actions = array of legal [x, y] actions
                                    //   config.actionCosts[i] = cost of config.actions[i]
                                    //   config.heuristic = 'diag', 'card', 'dist', or 'zero'
        this.name = "jps";
        this.grid = grid;           // the grid we are using to search
        this.sx = -1;               // x location of the start state
        this.sy = -1;               // y location of the start state
        this.gx = -1;               // x location of the goal state
        this.gy = -1;               // y location of the goal state
        this.size = 1;              // the square side length (size) of the agent
        this.maxSize = 3;           // the maximum size of an agent

        this.inProgress = false;    // whether the search is in progress
        this.expanded = 0;          // number of nodes expanded (drawn in GUI)

        this.map = [];
        this.path = [];             // the path, if the search found one
        this.open = [];             // the current open list of the search 
        this.closed = [];           // the current closed list of the search
        this.ignore = [];           // list that contains ignored nodes
        this.cost = 'Search Not Completed';   // the cost of the path found, -1 if no path
        this.objectSize= 1;
        this.computeSectors();
        this.currentNode;
    }
    
    //set up all the necessary data structures to begin a new search
    startSearch(sx, sy, gx, gy, size) {
        // deals with an edge-case with the GUI, leave this line here
        if (sx == -1 || gx == -1) { return; }
        this.cost = -1;

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
        let firstNode = new NodeV(sx, sy, null, null, 0, h, null);
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
        if ((this.map[size][x1][y1] == this.map[size][x2][y2]) && (this.map[size][x1][y1] != 0)){
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
        //if( this.grid.get(x,y) != this.grid.get(nx, ny) ){
        //    return false;
        //}
        if (action[0] == 0 || action[1] == 0) {
            if(this.isConnected(x,y,x,ny,size) && this.isConnected(x,y,nx,y,size)){
                return true;
            }
        }
        else{
            if(this.isConnected(x,y,nx,ny,size)){
                return true;
            }
        }
        return false;

    }
    //compute and store the connected sectors
    computeSectors() {
        //initialize empty sectors array
        this.map = [];
        for (let s = 0; s <= this.maxSize; s++) {
            let temp = [];
            for (let w = 0; w < this.grid.width; w++) {
                temp[w] = [];
                for (let h = 0; h < this.grid.height; h++) {
                    temp[w][h] = 0;
                }
            }
            this.map[s] = temp;
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
                        this.map[s][sectorOpen[i][0]][sectorOpen[i][1]] = this.sectorNumber[s];

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
        return this.map[size][x][y];
    }
    
    canFit(x, y, size) {
		for (let i = x; i < (size + x) - 1; i++) {
			for (let j = y; j < (size + y) - 1; j++) {
				if (this.grid.get(i, j) != this.grid.get(i+1, j) || this.grid.get(i, j) != this.grid.get(i, j+1) || this.grid.get(i, j) != this.grid.get(i+1, j+1)) {
					return false;
				}
			}
		}
    	return true;
    }

    
    //performs one iteration of search
    searchIteration() {
        // if we've already finished the search, do nothing
        if (!this.inProgress) { return; }
        //check if the start and end goals are connected
        if (!this.isConnected(this.sx, this.sy, this.gx, this.gy, this.objectSize)) {   
            this.inProgress = false; 
            return; 
        }
        //if open list is empty, do nothing
        if(this.open.length == 0){
            this.path = [];
            this.inProgress = false;
            return;
        }
        //remove the node from the open list
        let minIndex = this.getMinIndex(this.open);
        this.currentNode = this.open[minIndex];
        this.open.splice(minIndex,1);
        //if found the goal, return path
        if(this.findGoal(this.currentNode.x, this.currentNode.y)){
            var nodeOfPath = this.currentNode;
            this.cost = nodeOfPath.g;
            while (nodeOfPath.parent != null) {
                for (let i = 0; i < nodeOfPath.action.length; i++) {
                    this.path.splice(0, 0, nodeOfPath.action[i]);
                }
                nodeOfPath = nodeOfPath.parent;
            }
            this.inProgress = false;          
            return;
        }
        //check if the node is on the closed list
        if(this.checkClosed(this.currentNode)){
            return;
        }
        //add the node to the closed list
        this.addClosed(this.currentNode);
        
        if(this.currentNode.parent == null){
            var tempActions = this.config.actions;
        }
        else{
            // ignore certain actions
            var tempActions = this.ignoreActions(this.currentNode);
        }
        // search horizontal and vertical direction
        this.cardinalSearch(this.currentNode, tempActions);
        // search diagonal direction
        this.diagonalSearch(this.currentNode, tempActions);
    }

    // search in cardinal directions
    cardinalSearch(node, actions) {
        for (let a = 0; a < actions.length; a++) {
            // if the action is not diagonal
            if (actions[a][0] == 0 || actions[a][1] == 0) {
                var found  = false;
                var actionsOfPath = [];
                var curNode = node;
                while (!found) {
                    if (this.isLegalAction(curNode.x, curNode.y, this.objectSize, actions[a])) {
                        actionsOfPath.push(actions[a]);
                        var nNode = this.setNewNode(curNode, actionsOfPath, node);
                        curNode = nNode;
                        // add the node to the open list and stop searching if the node is the goal node
                        if (curNode.x == this.gx && curNode.y == this.gy) {
                            if (node.x == this.currentNode.x && node.y == this.currentNode.y) {
                                this.open.push(curNode);
                            }
                            else {
                                return true;
                            }
                            found = true;
                        }
                        // continue to search if the node is not the goal
                        else {
                            this.ignore.push(curNode);
                            // check if the node has a forced neighbor
                            var direction = this.findForcedNeighbour(curNode, actions[a]);   
                            if (direction != null) {
                                if (node.x == this.currentNode.x && node.y == this.currentNode.y) {
                                    curNode.direction = direction;
                                    this.open.push(curNode);
                                }
                                else {
                                    return true;
                                }
                                found = true;
                            }
                        }
                    }
                    // stop searching if the node encounters an obstcale
                    else {
                        found = true;
                    }
                }
            }
        }
        return false;       
    }
    //search diagonal direction
    diagonalSearch(node, actions) {
        for (let a = 0; a < actions.length; a++) {
            // if the action is diagonal 
            if ( actions[a][0] != 0 && actions[a][1] != 0 ) {
                var found = false;
                var actionsOfPath = [];
                var curNode = node;
                while (!found) {
                    if (this.isLegalAction(curNode.x, curNode.y, this.objectSize, actions[a])) {
                        actionsOfPath.push(actions[a]);   
                        let nNode = this.setNewNode(curNode, actionsOfPath, node);
                        curNode = nNode;
                        // add the node to the open list and stop searching if the node is the goal node
                        if (curNode.x == this.gx && curNode.y == this.gy) {                            
                            this.open.push(curNode);
                            found = true;
                        }
                        // continue to search if the node is not the goal
                        else {
                            this.ignore.push(curNode);
                            // check if the node has a forced neighbor
                            let direction = this.findForcedNeighbour(curNode, actions[a]);
                            
                            if (direction != null) {
                                curNode.direction = direction;
                                this.open.push(curNode);
                                found = true;
                            }
                            // search cardinal directions
                            var tempActions = [[actions[a][0], 0],[0, actions[a][1]]];
                            // add the node into the open list if find forced neighbor
                            if (this.cardinalSearch(curNode, tempActions)) {
                                curNode.direction =  actions[a];
                                this.open.push(curNode);
                                found = true;
                            } 
                        }
                    }
                    // if it reach to the bound or obstcale, stop the searching
                    else {
                        found = true;
                    }
                } 
            }
            
        }
    } 

    //find the forced neighbour
    findForcedNeighbour(node, action) {
        if (action[0] == 0) {
            if (!this.isLegalAction(node.x, node.y, this.objectSize, [1,0]) && this.isLegalAction(node.x, node.y, this.objectSize, [1,action[1]])) {
                return [1, action[1]];
            }
            else if (!this.isLegalAction(node.x, node.y, this.objectSize, [-1,0]) && this.isLegalAction(node.x, node.y, this.objectSize, [-1,action[1]])) {
                return [-1,action[1]];
            }
        }
        else if (action[1] == 0) {
            if (!this.isLegalAction(node.x, node.y, this.objectSize, [0,1]) && this.isLegalAction(node.x, node.y, this.objectSize, [action[0],1])) {
                return [action[0],1];
            } 
            else if (!this.isLegalAction(node.x, node.y, this.objectSize, [0,-1]) && this.isLegalAction(node.x, node.y, this.objectSize, [action[0],-1])) {
                return [action[0],-1];
            }
        }
        else {
            if (!this.isLegalAction(node.x, node.y, this.objectSize, [-action[0],0]) && this.isLegalAction(node.x, node.y, this.objectSize, [-action[0],action[1]])) {
                return [-action[0],action[1]];
            } 
            else if(!this.isLegalAction(node.x, node.y, this.objectSize, [0,-action[1]]) && this.isLegalAction(node.x, node.y, this.objectSize, [action[0],-action[1]])) {
                return [action[0],-action[1]];
            }
        }
        return null;
    }

    //construct a new node
    setNewNode(node, action, parent) {
        let nx = node.x + action[action.length - 1][0];
        let ny = node.y + action[action.length - 1][1];
        let currAction = [];
        let actionIndex = 0;
        for (var i = 0; i < this.config.actions.length; i++) {
            if (this.config.actions[i][1] == action[0][1] &&  this.config.actions[i][0] == action[0][0] ) {
                actionIndex = i;
            }
        }
        let g = this.config.actionCosts[actionIndex] * action.length;
        for (var i = 0; i < action.length; i++) {
            currAction.push(action[i]);  
        }
        let h = this.estimateCost(nx, ny, this.gx, this.gy);
        let nNode = new NodeV(nx, ny, parent, currAction, g, h, null);
        return nNode;
    }
    //jump certain nodes
    ignoreActions(child) {
        let actions = []
        let action1 = child.direction[0];
        let action2 = child.direction[1];
        actions.push([action1, action2]); 
        actions.push([action1, 0]);
        actions.push([0, action2]);
        return actions;
    }

    //return the ignored list
    getIgnore() {
        return this.ignore;
    }


    //return the open list states
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
    //return closed list
    getClosed(){
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
    //return the index of the node with the smallest heuristic value
    getMinIndex(list){
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
}

class NodeV {
    constructor(x, y, parent, action, g, h, direction) {
        this.x = x;
        this.y = y;
        this.action = action;
        this.parent = parent;
        this.g = g;
        this.h = h;
        this.direction = direction;

    }
}