class Pathfinding {
    constructor() {
        // Grid settings
        this.gridSize = 1.0; // 1 unit grid cells
        this.robotRadius = 1.2; // Slightly inflated for safety buffer
        this.grid = [];
        this.gridWidth = 0;
        this.gridHeight = 0;
        
        // World bounds
        this.worldBounds = {
            minX: -250,
            maxX: 250,
            minZ: -250,
            maxZ: 250
        };
    }
    
    // Initialize navigation grid
    createNavigationGrid() {
        this.gridWidth = Math.ceil((this.worldBounds.maxX - this.worldBounds.minX) / this.gridSize);
        this.gridHeight = Math.ceil((this.worldBounds.maxZ - this.worldBounds.minZ) / this.gridSize);
        
        // Initialize grid as walkable
        this.grid = [];
        for (let x = 0; x < this.gridWidth; x++) {
            this.grid[x] = [];
            for (let z = 0; z < this.gridHeight; z++) {
                this.grid[x][z] = 0; // 0 = walkable, 1 = obstacle
            }
        }
        
        console.log(`Navigation grid created: ${this.gridWidth}x${this.gridHeight} (${this.gridSize} unit cells)`);
        console.log(`Robot radius: ${this.robotRadius} units`);
    }
    
    // Mark room as obstacle with minimal robot clearance
    markRoomAsObstacle(room) {
        // Minimal clearance - just enough for robot to not touch walls
        const clearance = this.robotRadius + 0.2; // Reduced from +0.5 to +0.2
        
        const startX = Math.floor((room.position.x - room.size.x/2 - clearance - this.worldBounds.minX) / this.gridSize);
        const endX = Math.ceil((room.position.x + room.size.x/2 + clearance - this.worldBounds.minX) / this.gridSize);
        const startZ = Math.floor((room.position.z - room.size.z/2 - clearance - this.worldBounds.minZ) / this.gridSize);
        const endZ = Math.ceil((room.position.z + room.size.z/2 + clearance - this.worldBounds.minZ) / this.gridSize);
        
        // Clamp to grid bounds
        const clampedStartX = Math.max(0, startX);
        const clampedEndX = Math.min(this.gridWidth - 1, endX);
        const clampedStartZ = Math.max(0, startZ);
        const clampedEndZ = Math.min(this.gridHeight - 1, endZ);
        
        console.log(`Marking room as obstacle: ${room.position.x.toFixed(1)}, ${room.position.z.toFixed(1)}`);
        console.log(`Grid range with clearance: X[${clampedStartX}, ${clampedEndX}], Z[${clampedStartZ}, ${clampedEndZ}]`);
        
        // Mark room area as obstacle
        for (let x = clampedStartX; x < clampedEndX; x++) {
            for (let z = clampedStartZ; z < clampedEndZ; z++) {
                this.grid[x][z] = 1;
            }
        }
        
        console.log(`Marked ${(clampedEndX - clampedStartX) * (clampedEndZ - clampedStartZ)} grid cells as obstacles`);
    }
    
    // Mark furniture as obstacle with minimal robot clearance
    markFurnitureAsObstacle(position, size) {
        const clearance = this.robotRadius + 0.1; // Reduced from +0.3 to +0.1
        
        const startX = Math.floor((position.x - size.x/2 - clearance - this.worldBounds.minX) / this.gridSize);
        const endX = Math.ceil((position.x + size.x/2 + clearance - this.worldBounds.minX) / this.gridSize);
        const startZ = Math.floor((position.z - size.z/2 - clearance - this.worldBounds.minZ) / this.gridSize);
        const endZ = Math.ceil((position.z + size.z/2 + clearance - this.worldBounds.minZ) / this.gridSize);
        
        const clampedStartX = Math.max(0, startX);
        const clampedEndX = Math.min(this.gridWidth - 1, endX);
        const clampedStartZ = Math.max(0, startZ);
        const clampedEndZ = Math.min(this.gridHeight - 1, endZ);
        
        console.log(`Marking furniture as obstacle: (${position.x.toFixed(1)}, ${position.z.toFixed(1)}) size (${size.x.toFixed(1)}, ${size.z.toFixed(1)})`);
        console.log(`Grid range with clearance: X[${clampedStartX}, ${clampedEndX}], Z[${clampedStartZ}, ${clampedEndZ}]`);
        
        for (let x = clampedStartX; x < clampedEndX; x++) {
            for (let z = clampedStartZ; z < clampedEndZ; z++) {
                this.grid[x][z] = 1;
            }
        }
        
        console.log(`Marked ${(clampedEndX - clampedStartX) * (clampedEndZ - clampedStartZ)} grid cells as furniture obstacles`);
    }
    
    // Convert world coordinates to grid coordinates
    worldToGrid(worldPos) {
        const gridX = Math.floor((worldPos.x - this.worldBounds.minX) / this.gridSize);
        const gridZ = Math.floor((worldPos.z - this.worldBounds.minZ) / this.gridSize);
        return { x: gridX, z: gridZ };
    }
    
    // Convert grid coordinates to world coordinates
    gridToWorld(gridPos) {
        const worldX = this.worldBounds.minX + (gridPos.x + 0.5) * this.gridSize;
        const worldZ = this.worldBounds.minZ + (gridPos.z + 0.5) * this.gridSize;
        return new THREE.Vector3(worldX, 0, worldZ);
    }
    
    // Check if a grid node is valid and walkable
    isValidNode(node) {
        if (node.x < 0 || node.x >= this.gridWidth || node.z < 0 || node.z >= this.gridHeight) {
            return false;
        }
        return this.grid[node.x][node.z] === 0;
    }
    
    // Enhanced A* pathfinding with strict obstacle avoidance
    findPath(start, end) {
        const startNode = this.worldToGrid(start);
        const endNode = this.worldToGrid(end);
        
        console.log(`Pathfinding: Start (${start.x.toFixed(1)}, ${start.z.toFixed(1)}) -> Grid(${startNode.x}, ${startNode.z})`);
        console.log(`Pathfinding: End (${end.x.toFixed(1)}, ${end.z.toFixed(1)}) -> Grid(${endNode.x}, ${endNode.z})`);
        
        if (!this.isValidNode(startNode) || !this.isValidNode(endNode)) {
            console.log('Start or end position is invalid');
            return null;
        }
        
        // A* algorithm with strict obstacle validation
        const openSet = [startNode];
        const closedSet = new Set();
        const cameFrom = new Map();
        const gScore = new Map();
        const fScore = new Map();
        
        gScore.set(this.nodeToString(startNode), 0);
        fScore.set(this.nodeToString(startNode), this.heuristic(startNode, endNode));
        
        let iterations = 0;
        const maxIterations = 50000; // Increased from 10000 to handle complex paths
        
        while (openSet.length > 0 && iterations < maxIterations) {
            iterations++;
            
            // Find node with lowest fScore
            let current = openSet[0];
            let currentIndex = 0;
            for (let i = 1; i < openSet.length; i++) {
                if (fScore.get(this.nodeToString(openSet[i])) < fScore.get(this.nodeToString(current))) {
                    current = openSet[i];
                    currentIndex = i;
                }
            }
            
            if (this.nodesEqual(current, endNode)) {
                const path = this.reconstructPath(cameFrom, current);
                console.log(`Found path with ${path.length} waypoints using A* (${iterations} iterations)`);
                
                // CRITICAL: Validate that the entire path is walkable
                if (!this.validatePath(path)) {
                    console.log('ERROR: A* generated invalid path through obstacles!');
                    return null;
                }
                
                return path;
            }
            
            openSet.splice(currentIndex, 1);
            closedSet.add(this.nodeToString(current));
            
            // Get neighbors with strict validation
            const neighbors = this.getNeighbors(current);
            for (const neighbor of neighbors) {
                // CRITICAL: Double-check neighbor is actually walkable
                if (!this.isValidNode(neighbor)) {
                    console.log(`WARNING: Invalid neighbor detected: (${neighbor.x}, ${neighbor.z})`);
                    continue;
                }
                
                if (closedSet.has(this.nodeToString(neighbor))) {
                    continue;
                }
                
                const tentativeGScore = gScore.get(this.nodeToString(current)) + this.getMoveCost(current, neighbor);
                const neighborKey = this.nodeToString(neighbor);
                
                if (!openSet.some(n => this.nodesEqual(n, neighbor))) {
                    openSet.push(neighbor);
                } else if (tentativeGScore >= gScore.get(neighborKey)) {
                    continue;
                }
                
                cameFrom.set(neighborKey, current);
                gScore.set(neighborKey, tentativeGScore);
                fScore.set(neighborKey, tentativeGScore + this.heuristic(neighbor, endNode));
            }
        }
        
        if (iterations >= maxIterations) {
            console.log('Pathfinding exceeded maximum iterations');
        } else {
            console.log('No path found - target unreachable');
        }
        return null;
    }
    
    // Get neighbors with strict validation - 4-directional only to prevent corner cutting
    getNeighbors(node) {
        const neighbors = [];
        const directions = [
            {x: 1, z: 0}, {x: -1, z: 0}, {x: 0, z: 1}, {x: 0, z: -1}  // 4-directional only
        ];
        
        for (const dir of directions) {
            const neighbor = {x: node.x + dir.x, z: node.z + dir.z};
            // CRITICAL: Only add neighbors that are actually walkable
            if (this.isValidNode(neighbor)) {
                neighbors.push(neighbor);
            }
        }
        
        return neighbors;
    }
    
    // Get move cost - all moves cost 1 since we're 4-directional only
    getMoveCost(from, to) {
        return 1.0; // All moves cost the same
    }
    
    // Heuristic function (Euclidean distance)
    heuristic(a, b) {
        const dx = a.x - b.x;
        const dz = a.z - b.z;
        return Math.sqrt(dx * dx + dz * dz);
    }
    
    // Reconstruct path from cameFrom map
    reconstructPath(cameFrom, current) {
        const path = [this.gridToWorld(current)];
        let currentKey = this.nodeToString(current);
        
        while (cameFrom.has(currentKey)) {
            current = cameFrom.get(currentKey);
            path.unshift(this.gridToWorld(current));
            currentKey = this.nodeToString(current);
        }
        
        return path;
    }
    
    // Helper methods
    nodesEqual(a, b) {
        return a.x === b.x && a.z === b.z;
    }
    
    nodeToString(node) {
        return `${node.x},${node.z}`;
    }
    
    // Check if a world position is valid
    isValidPosition(worldPos) {
        const gridPos = this.worldToGrid(worldPos);
        if (!this.isValidNode(gridPos)) {
            return false;
        }
        return this.grid[gridPos.x][gridPos.z] === 0;
    }
    
    // Validate that a path doesn't go through obstacles
    validatePath(path) {
        if (!path || path.length === 0) return false;
        
        for (let i = 0; i < path.length; i++) {
            if (!this.isValidPosition(path[i])) {
                console.log(`Path validation failed at waypoint ${i}:`, path[i]);
                return false;
            }
        }
        return true;
    }
    
    // Debug method to check grid state at specific positions
    debugGridPosition(worldPos) {
        const gridPos = this.worldToGrid(worldPos);
        const gridValue = this.grid[gridPos.x][gridPos.z];
        console.log(`Grid debug: World(${worldPos.x.toFixed(1)}, ${worldPos.z.toFixed(1)}) -> Grid(${gridPos.x}, ${gridPos.z}) = ${gridValue} (${gridValue === 0 ? 'walkable' : 'obstacle'})`);
        return gridValue;
    }
    
    // Debug method to show a sample of the grid around a position
    debugGridAround(worldPos, radius = 5) {
        const centerGrid = this.worldToGrid(worldPos);
        console.log(`Grid sample around (${worldPos.x.toFixed(1)}, ${worldPos.z.toFixed(1)}) -> Grid(${centerGrid.x}, ${centerGrid.z}):`);
        
        for (let z = centerGrid.z - radius; z <= centerGrid.z + radius; z++) {
            if (z < 0 || z >= this.gridHeight) continue;
            
            let row = '';
            for (let x = centerGrid.x - radius; x <= centerGrid.x + radius; x++) {
                if (x < 0 || x >= this.gridWidth) {
                    row += ' ';
                } else if (x === centerGrid.x && z === centerGrid.z) {
                    row += 'X'; // Current position
                } else {
                    row += this.grid[x][z] === 0 ? '.' : '#';
                }
            }
            console.log(`Z${z.toString().padStart(3)}: ${row}`);
        }
    }
    
    // Debug method to visualize a path on the grid
    debugPathOnGrid(path, radius = 8) {
        if (!path || path.length === 0) return;
        
        // Find the center of the path
        let centerX = 0, centerZ = 0;
        path.forEach(pos => {
            const grid = this.worldToGrid(pos);
            centerX += grid.x;
            centerZ += grid.z;
        });
        centerX = Math.floor(centerX / path.length);
        centerZ = Math.floor(centerZ / path.length);
        
        console.log(`=== PATH VISUALIZATION (${path.length} waypoints) ===`);
        console.log(`Path center: Grid(${centerX}, ${centerZ})`);
        
        for (let z = centerZ - radius; z <= centerZ + radius; z++) {
            if (z < 0 || z >= this.gridHeight) continue;
            
            let row = '';
            for (let x = centerX - radius; x <= centerX + radius; x++) {
                if (x < 0 || x >= this.gridWidth) {
                    row += ' ';
                } else {
                    // Check if this grid position is part of the path
                    let isPath = false;
                    let pathIndex = -1;
                    for (let i = 0; i < path.length; i++) {
                        const pathGrid = this.worldToGrid(path[i]);
                        if (pathGrid.x === x && pathGrid.z === z) {
                            isPath = true;
                            pathIndex = i;
                            break;
                        }
                    }
                    
                    if (isPath) {
                        row += pathIndex.toString().padStart(1, '0'); // Show path order
                    } else if (this.grid[x][z] === 0) {
                        row += '.';
                    } else {
                        row += '#';
                    }
                }
            }
            console.log(`Z${z.toString().padStart(3)}: ${row}`);
        }
        console.log('=== END PATH VISUALIZATION ===');
    }

    // Find nearest walkable position from a given position
    findNearestWalkablePosition(startPos, maxRadius = 10) {
        const startGrid = this.worldToGrid(startPos);
        
        // Search in expanding circles for a walkable position
        for (let radius = 1; radius <= maxRadius; radius++) {
            for (let dx = -radius; dx <= radius; dx++) {
                for (let dz = -radius; dz <= radius; dz++) {
                    // Only check positions at the current radius (perimeter)
                    if (Math.abs(dx) === radius || Math.abs(dz) === radius) {
                        const testGrid = {
                            x: startGrid.x + dx,
                            z: startGrid.z + dz
                        };
                        
                        if (this.isValidNode(testGrid)) {
                            const walkablePos = this.gridToWorld(testGrid);
                            console.log(`Found walkable position at radius ${radius}: (${walkablePos.x.toFixed(1)}, ${walkablePos.z.toFixed(1)})`);
                            return walkablePos;
                        }
                    }
                }
            }
        }
        
        console.log('No walkable position found within radius', maxRadius);
        return null;
    }

    // Check line-of-sight between two grid positions (prevents corner-cutting)
    hasLineOfSight(ax, az, bx, bz) {
        const dx = Math.abs(bx - ax);
        const dz = Math.abs(bz - az);
        const sx = ax < bx ? 1 : -1;
        const sz = az < bz ? 1 : -1;
        let err = dx - dz;
        
        let x = ax, z = az;
        
        while (true) {
            // Current cell must be walkable
            if (!this.isValidNode({x, z})) return false;
            
            if (x === bx && z === bz) break;
            
            const e2 = 2 * err;
            // Track whether we step in x and/or z this iteration
            let stepX = false, stepZ = false;
            if (e2 > -dz) { err -= dz; x += sx; stepX = true; }
            if (e2 <  dx) { err += dx; z += sz; stepZ = true; }
            
            // If we advanced diagonally across a cell corner, enforce corner rule:
            if (stepX && stepZ) {
                // Both orthogonal neighbors at the shared corner must be walkable
                if (!this.isValidNode({x: x,       z: z - sz})) return false;
                if (!this.isValidNode({x: x - sx,  z: z      })) return false;
            }
        }
        
        return true;
    }
    
    // Short-range repulsion to avoid hugging obstacle edges
    shortRangeRepulsion(worldPos) {
        // sample a 3x3 grid around the robot and push away from blocked cells
        const g = this.worldToGrid(worldPos);
        let rx = 0, rz = 0;
        for (let dz = -1; dz <= 1; dz++) {
            for (let dx = -1; dx <= 1; dx++) {
                if (dx === 0 && dz === 0) continue;
                const nx = g.x + dx, nz = g.z + dz;
                if (nx < 0 || nz < 0 || nx >= this.gridWidth || nz >= this.gridHeight) continue;
                if (this.grid[nx][nz] === 1) { rx -= dx; rz -= dz; }
            }
        }
        const v = new THREE.Vector3(rx, 0, rz);
        if (v.lengthSq() > 0) v.normalize().multiplyScalar(0.25); // small bias
        return v;
    }
}
