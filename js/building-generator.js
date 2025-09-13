class BuildingGenerator {
    constructor(scene, pathfinding) {
        this.scene = scene;
        this.pathfinding = pathfinding;
        this.building = null;
        this.rooms = {};
        this.walls = [];
        this.furniture = [];
    }
    
    // Create the entire building
    createBuilding() {
        this.building = new THREE.Group();
        
        // Just create rooms - no corridors for now
        this.createRooms();
        this.createFurniture();
        
        this.scene.add(this.building);
        
        // Enable soft shadows and ensure PBR materials
        this.building.traverse((obj) => {
            if (obj && obj.isMesh) {
                obj.castShadow = true;
                obj.receiveShadow = true;
                // Ensure PBR responds well to lights
                if (obj.material && obj.material.isMaterial) {
                    if ('roughness' in obj.material && typeof obj.material.roughness !== 'number') obj.material.roughness = 0.9;
                    if ('metalness' in obj.material && typeof obj.material.metalness !== 'number') obj.material.metalness = 0.0;
                }
            }
        });
        
        console.log('Building created with', Object.keys(this.rooms).length, 'rooms');
    }
    
    // Create Valparaiso University engineering facilities
    createRooms() {
        this.rooms = {
            'GEMC': { 
                position: new THREE.Vector3(-80, 0, -80), 
                size: new THREE.Vector3(20, 6, 15),
                entrance: new THREE.Vector3(-80, 0, -67.5), // South entrance
                description: 'Gellersen Engineering & Mathematics Center - Main engineering building'
            },
            'Fites': { 
                position: new THREE.Vector3(-50, 0, -80), 
                size: new THREE.Vector3(15, 6, 12),
                entrance: new THREE.Vector3(-50, 0, -68), // South entrance
                description: 'Donald V. Fites Engineering Innovation Center - Engineering labs'
            },
            'SERF': { 
                position: new THREE.Vector3(-20, 0, -80), 
                size: new THREE.Vector3(18, 6, 14),
                entrance: new THREE.Vector3(-20, 0, -66), // South entrance
                description: 'James S. Markiewicz Solar Energy Research Facility - Solar furnace research'
            },
            'Hesse': { 
                position: new THREE.Vector3(20, 0, -80), 
                size: new THREE.Vector3(12, 6, 10),
                entrance: new THREE.Vector3(20, 0, -70), // South entrance
                description: 'Hesse Learning Center - Engineering tutoring center'
            },
            'iHub': { 
                position: new THREE.Vector3(50, 0, -80), 
                size: new THREE.Vector3(16, 6, 12),
                entrance: new THREE.Vector3(50, 0, -68), // South entrance
                description: 'Innovation Hub Makerspace - 3D printers, laser cutters, prototyping'
            },
            'Library': { 
                position: new THREE.Vector3(80, 0, -80), 
                size: new THREE.Vector3(25, 6, 20),
                entrance: new THREE.Vector3(80, 0, -60), // South entrance
                description: 'Christopher Center for Library & Information Resources - Main library'
            },
            'Chapel': { 
                position: new THREE.Vector3(-80, 0, -40), 
                size: new THREE.Vector3(20, 6, 15),
                entrance: new THREE.Vector3(-80, 0, -27.5), // South entrance
                description: 'Chapel of the Resurrection + Brandt Campanile - Iconic campus landmark'
            },
            'Science': { 
                position: new THREE.Vector3(-50, 0, -40), 
                size: new THREE.Vector3(15, 6, 12),
                entrance: new THREE.Vector3(-50, 0, -28), // South entrance
                description: 'Science Complex (CFS + NSC) - Chemistry, Biology, Biochemistry labs'
            },
            'Meteorology': { 
                position: new THREE.Vector3(-20, 0, -40), 
                size: new THREE.Vector3(18, 6, 14),
                entrance: new THREE.Vector3(-20, 0, -26), // North entrance
                description: 'Kallay-Christopher Hall - Meteorology program, weather observation deck'
            },
            'ARC': { 
                position: new THREE.Vector3(20, 0, -40), 
                size: new THREE.Vector3(25, 6, 20),
                entrance: new THREE.Vector3(20, 0, -20), // South entrance
                description: 'Athletics-Recreation Center - Basketball arena, gym, pool'
            },
            'Museum': { 
                position: new THREE.Vector3(50, 0, -40), 
                size: new THREE.Vector3(20, 6, 15),
                entrance: new THREE.Vector3(50, 0, -25), // South entrance
                description: 'Brauer Museum of Art - University art museum'
            },
            'Weseman': { 
                position: new THREE.Vector3(80, 0, -40), 
                size: new THREE.Vector3(12, 6, 10),
                entrance: new THREE.Vector3(80, 0, -30), // South entrance
                description: 'Weseman Hall - Academic building'
            },
            'Lebien': { 
                position: new THREE.Vector3(-80, 0, 0), 
                size: new THREE.Vector3(20, 6, 15),
                entrance: new THREE.Vector3(-80, 0, 12.5), // North entrance
                description: 'Lebien Hall - Academic building'
            },
            'Lankenau': { 
                position: new THREE.Vector3(-50, 0, 0), 
                size: new THREE.Vector3(18, 6, 14),
                entrance: new THREE.Vector3(-50, 0, 14), // North entrance
                description: 'Lankenau Hall - Academic building'
            },
            'Alumni': { 
                position: new THREE.Vector3(-20, 0, 0), 
                size: new THREE.Vector3(18, 6, 14),
                entrance: new THREE.Vector3(-20, 0, 14), // North entrance
                description: 'Alumni Hall - Academic building'
            },
            'Brandt': { 
                position: new THREE.Vector3(20, 0, 0), 
                size: new THREE.Vector3(20, 6, 15),
                entrance: new THREE.Vector3(20, 0, 15), // North entrance
                description: 'Brandt Hall - Academic building'
            },
            'Founders': { 
                position: new THREE.Vector3(50, 0, 0), 
                size: new THREE.Vector3(22, 6, 16),
                entrance: new THREE.Vector3(50, 0, 16), // North entrance
                description: 'Founder\'s Hall - Academic building'
            },
            'Kretzmann': { 
                position: new THREE.Vector3(80, 0, 0), 
                size: new THREE.Vector3(18, 6, 14),
                entrance: new THREE.Vector3(80, 0, 14), // North entrance
                description: 'Kretzmann Hall - Academic building'
            },
            'Nielsen': { 
                position: new THREE.Vector3(-80, 0, 40), 
                size: new THREE.Vector3(20, 6, 15),
                entrance: new THREE.Vector3(-80, 0, 52.5), // North entrance
                description: 'Nielsen Institute - Academic building'
            },
            'Ateufack': { 
                position: new THREE.Vector3(-50, 0, 40), 
                size: new THREE.Vector3(18, 6, 14),
                entrance: new THREE.Vector3(-50, 0, 54), // North entrance
                description: 'Ateufack School of AI - Academic building'
            }
        };
        
        // Create room geometries and add to pathfinding grid
        Object.keys(this.rooms).forEach(roomKey => {
            const room = this.rooms[roomKey];
            this.createRoomMesh(roomKey, room);
            // Mark room as obstacle AFTER corridors are already marked
            this.pathfinding.markRoomAsObstacle(room);
        });
    }
    
    // Create individual room mesh
    createRoomMesh(roomKey, room) {
        const geometry = new THREE.BoxGeometry(room.size.x, room.size.y, room.size.z);
        const material = new THREE.MeshStandardMaterial({ 
            color: this.getRoomColor(roomKey),
            transparent: true,
            opacity: 0.7,
            roughness: 0.9,
            metalness: 0.0
        });
        const mesh = new THREE.Mesh(geometry, material);
        mesh.position.copy(room.position);
        mesh.position.y = room.size.y / 2;
        mesh.castShadow = true;
        mesh.receiveShadow = true;
        
        // Add room label
        const label = this.createRoomLabel(roomKey, room.position);
        if (label) this.building.add(label);
        
        // Add entrance marker
        this.createEntranceMarker(room.entrance);
        
        this.building.add(mesh);
    }
    
    // Create entrance marker (small red sphere)
    createEntranceMarker(entrancePosition) {
        const markerGeometry = new THREE.SphereGeometry(0.5, 8, 8);
        const markerMaterial = new THREE.MeshStandardMaterial({ 
            color: 0xff0000,
            roughness: 0.3,
            metalness: 0.0,
            emissive: 0x330000
        });
        const marker = new THREE.Mesh(markerGeometry, markerMaterial);
        marker.position.copy(entrancePosition);
        marker.position.y = 0.5; // Slightly above ground
        this.scene.add(marker);
    }
    
    // Create furniture
    createFurniture() {
        const furnitureMaterial = new THREE.MeshStandardMaterial({ 
            color: 0x8B4513,
            roughness: 0.9,
            metalness: 0.0
        });
        
        // Add desks to academic buildings
        ['GEMC', 'Fites', 'Hesse', 'iHub', 'Science', 'Meteorology'].forEach(roomKey => {
            const room = this.rooms[roomKey];
            if (room) {
                const desk = this.addDesk(room.position, furnitureMaterial);
                // Mark desk as obstacle in pathfinding grid
                this.pathfinding.markFurnitureAsObstacle(desk.position, new THREE.Vector3(1.5, 0.8, 0.8));
            }
        });
        
        // Add tables to common areas
        ['Library', 'Chapel', 'ARC', 'Museum'].forEach(roomKey => {
            const room = this.rooms[roomKey];
            if (room) {
                const table = this.addTable(room.position, furnitureMaterial);
                // Mark table as obstacle in pathfinding grid
                this.pathfinding.markFurnitureAsObstacle(table.position, new THREE.Vector3(2, 0.8, 1.2));
            }
        });
    }
    
    // Add desk to room
    addDesk(position, material) {
        const deskGeometry = new THREE.BoxGeometry(1.5, 0.8, 0.8);
        const desk = new THREE.Mesh(deskGeometry, material);
        desk.position.copy(position);
        desk.position.y = 0.4;
        desk.position.x += 2;
        desk.castShadow = true;
        this.scene.add(desk);
        this.furniture.push(desk);
        return desk; // Return for pathfinding integration
    }
    
    // Add table to room
    addTable(position, material) {
        const tableGeometry = new THREE.BoxGeometry(2, 0.8, 1.2);
        const table = new THREE.Mesh(tableGeometry, material);
        table.position.copy(position);
        table.position.y = 0.4;
        table.position.x += 2;
        table.castShadow = true;
        this.scene.add(table);
        this.furniture.push(table);
        return table; // Return for pathfinding integration
    }
    
    // Create room label
    createRoomLabel(roomKey, position) {
        try {
            const canvas = document.createElement('canvas');
            const context = canvas.getContext('2d');
            canvas.width = 128;
            canvas.height = 64;
            
            context.fillStyle = 'white';
            context.fillRect(0, 0, 128, 64);
            context.fillStyle = 'black';
            context.font = 'bold 16px Arial';
            context.textAlign = 'center';
            context.fillText(roomKey, 64, 40);
            
            const texture = new THREE.CanvasTexture(canvas);
            const material = new THREE.MeshBasicMaterial({ map: texture, transparent: true });
            const geometry = new THREE.PlaneGeometry(8, 4);
            const label = new THREE.Mesh(geometry, material);
            
            label.position.copy(position);
            label.position.y = 8;
            label.position.z += 8;
            
            return label;
        } catch (error) {
            console.error('Error creating room label:', error);
            return null;
        }
    }
    
    // Get room color - updated for Valparaiso University theme
    getRoomColor(roomKey) {
        const colors = {
            // Main engineering facilities (primary colors)
            'GEMC': 0x2E86AB, 'Fites': 0xA23B72, 'SERF': 0xF18F01, 'Hesse': 0xC73E1D, 'iHub': 0x7209B7,
            // Academic buildings (secondary colors)
            'Library': 0x4ECDC4, 'Chapel': 0xFF6B6B, 'Science': 0x45B7D1, 'Meteorology': 0xFF6348,
            'ARC': 0x2ED573, 'Museum': 0xFF9FF3, 'Weseman': 0x54A0FF, 'Lebien': 0xFF6B6B,
            'Lankenau': 0x5F27CD, 'Alumni': 0xFFA502, 'Brandt': 0x70A1FF, 'Founders': 0xFF9FF3,
            'Kretzmann': 0x2ED573, 'Nielsen': 0x54A0FF, 'Ateufack': 0x7209B7
        };
        return colors[roomKey] || 0xcccccc;
    }
    
    // Get rooms object
    getRooms() {
        return this.rooms;
    }
}
