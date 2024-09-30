#include "cells.h"
#include "fifo.h"

extern FIFO_buffer_t queue  ;
extern cell_t * stp ;

extern cell_t g_maze[MAZE_SIZE][MAZE_SIZE];
void log(char* text , int val) {
    fprintf(stderr, "%s %d\n", text ,val);
    fflush(stderr);
}

extern robot_position_t head_pos ;
void init_all_cells(cell_t maze[MAZE_SIZE][MAZE_SIZE], uint16_t size) {
    // Directions for moving up, down, left, right
    int directions[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

    // Initialize all cells to a large value (e.g., -1) indicating unvisited
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            maze[i][j].cell_val = -1;
            maze[i][j].row_position = i;
            maze[i][j].col_position = j;

            // Set default wall statuses to no_wall
            maze[i][j].front_wall = no_wall;
            maze[i][j].right_wall = no_wall;
            maze[i][j].left_wall = no_wall;
            maze[i][j].back_wall = no_wall;
            // Check if the cell is on the boundary of the maze
            if (i == 0) {
                maze[i][j].front_wall = there_wall; // Top boundary
            }
            if (i == size - 1) {
                maze[i][j].back_wall = there_wall;  // Bottom boundary
            }
            if (j == 0) {
                maze[i][j].left_wall = there_wall;  // Left boundary
            }
            if (j == size - 1) {
                maze[i][j].right_wall = there_wall; // Right boundary
            }
        }
    }

    // Set the center cells to 0
    //                   4    {{7, 7}, {7, 8}, {8, 7}, {8, 8}};
    point_t start_points[4] = {{1, 1}, {1, 2}, {2, 1}, {2, 2}};
    //                  4
    for (int i = 0; i < 4; i++) {
        int x = start_points[i].x;
        int y = start_points[i].y;
        maze[x][y].cell_val = 0;
       
    }

    // Queue for BFS
    point_t queue[MAZE_SIZE * MAZE_SIZE];
    int front = 0, rear = 0;

    // Enqueue the center points
    //                  4
    for (int i = 0; i < 4; i++) {
        queue[rear++] = start_points[i];
    }

    // BFS to fill in the values
    while (front < rear) {
        point_t current = queue[front++];
        int current_val = maze[current.x][current.y].cell_val;
        //                  4
        for (int i = 0; i < 4; i++) {
            int new_row = current.x + directions[i][0];
            int new_col = current.y + directions[i][1];

            if (new_row >= 0 && new_row < size && new_col >= 0 && new_col < size) {
                if (maze[new_row][new_col].cell_val == -1) {  // If the cell is unvisited
                    maze[new_row][new_col].cell_val = current_val + 1;
                  
                    queue[rear++] = (point_t){new_row, new_col};
                }
            }
        }
    }
}
void goto_smallest_val(cell_t maze [MAZE_SIZE][MAZE_SIZE], cell_t * current_cell){
		// Directions for moving up, left, down, right
			if(stp->cell_val!=0){
					set_cell_walls(maze,current_cell);
			        log("current row : ",current_cell->row_position);
			        log("current col : ",current_cell->col_position);
				  	int directions[4][2] = {{-1, 0}, {0, -1}, {1, 0}, {0, 1}};
				    wall_status walls[4] = {(wall_status)current_cell->front_wall,
				                              (wall_status)current_cell->left_wall,
				                              (wall_status)current_cell->back_wall,
				                              (wall_status)current_cell->right_wall};
				    /*        	current_cell->back_wall = front_status;
			        			current_cell->left_wall = right_status;
			        			current_cell->right_wall = left_status;

			        */
				    uint32_t min_neighbor_val = UINT_MAX;
				    cell_t *min_neighbor_cell = NULL;
				    int all_neighbors_greater = 1;
				    // Check accessible neighboring cells
				    for (int i = 0; i < 4; i++) {
				        int new_row = current_cell->row_position + directions[i][0];
				        int new_col = current_cell->col_position + directions[i][1];
				        log( "checking on : " ,i);
				        log("new row : ",new_row);
				        log("new col : ",new_col);
				        // Check if the new cell position is within bounds
				        if (new_row >= 0 && new_row < MAZE_SIZE && new_col >= 0 && new_col < MAZE_SIZE) {
				            // Check if there is no wall between current cell and the new cell
				            if (walls[i] == no_wall) {
				            	log( "no wall : ",1 );
				                cell_t *neighbor_cell = &maze[new_row][new_col];
				                if (neighbor_cell->cell_val < min_neighbor_val) {
				                    min_neighbor_val = neighbor_cell->cell_val;
				                    min_neighbor_cell = neighbor_cell;
				                }
				                if (neighbor_cell->cell_val < current_cell->cell_val) {
				                    // If any neighbor has a cell_val less than or equal to current_cell's cell_val
				                    all_neighbors_greater = 0;
				                }
				            }
				        }


				    }
			        log("min neighbor row : ",min_neighbor_cell->row_position);
			        log("min neighbor row col : ",min_neighbor_cell->col_position);
			        log("---------------------------------------- ",1);
				    // If all accessible neighbors have cell_val greater than the smallest neighbor's cell_val
				    if (all_neighbors_greater ) {
				       // update
				        log("enter update condition  : ",1);
				    	FIFO_enqueue(&queue,(*current_cell));
				    	update_maze_val ( g_maze );

				    }
				    else if((all_neighbors_greater==0)&&(FIFO_is_empty(&queue)== FIFO_empty) )
				    {
				    	// move to smallest value cell

				    	move_cell( current_cell , min_neighbor_cell);
						stp=min_neighbor_cell;

				    }
			}


}
void set_cell_walls(cell_t maze[MAZE_SIZE][MAZE_SIZE], cell_t *current_cell) {
   //  Simulated sensor functions (replace these with actual sensor reading functions)
    wall_status front_status = (wall_status)readFrontIR();  // Replace with actual sensor reading
    wall_status right_status =(wall_status) readRightIR();  // Replace with actual sensor reading
    wall_status left_status =(wall_status) readLeftIR();    // Replace with actual sensor reading

    // Update the wall status in the current cell
    if(head_pos== head_to_right)
    {
        current_cell->right_wall = front_status;
        current_cell->front_wall = left_status;
        current_cell->back_wall = right_status;
        current_cell->left_wall = no_wall;
    }
    else if(head_pos== head_to_left)
    {
        current_cell->left_wall = front_status;
        current_cell->front_wall = right_status;
        current_cell->back_wall = left_status;
        current_cell->right_wall = no_wall;
    }
    else if (head_pos == head_to_up)
    {
        current_cell->right_wall = right_status;
        current_cell->front_wall = front_status;
        current_cell->left_wall = left_status;
        current_cell->back_wall = no_wall;
    }
    else if(head_pos == head_to_down)
    {
        current_cell->back_wall = front_status;
        current_cell->left_wall = right_status;
        current_cell->right_wall = left_status;
        current_cell->front_wall = no_wall;
    }
}

void update_maze_val (cell_t maze [MAZE_SIZE][MAZE_SIZE]){
	 log("enter update function",1);
	cell_t temp ;
	while(!(FIFO_is_empty(&queue)== FIFO_empty))
	{
		 log("enter update while loop",1);
		FIFO_dequeue(&queue,&temp);
		//                               {0, -1} i1
	  	int directions[4][2] = {{-1, 0}, {0, -1}, {1, 0}, {0, 1}};
	    wall_status walls[4] = {(wall_status)maze[temp.row_position][temp.col_position].front_wall,
	    					(wall_status)	maze[temp.row_position][temp.col_position].left_wall,
							(wall_status)	maze[temp.row_position][temp.col_position].back_wall,
							(wall_status)	maze[temp.row_position][temp.col_position].right_wall};
			    uint32_t min_neighbor_val = UINT_MAX;
			    int all_neighbors_greater = 1;
			    for (int i = 0; i < 4; i++)
			    {
					int new_row = maze[temp.row_position][temp.col_position].row_position + directions[i][0];
					int new_col = maze[temp.row_position][temp.col_position].col_position + directions[i][1];
					// Check if the new cell position is within bounds
					if (new_row >= 0 && new_row < MAZE_SIZE && new_col >= 0 && new_col < MAZE_SIZE)
					{
						// Check if there is no wall between current cell and the new cell
						if (walls[i] == no_wall)
						{
							cell_t *neighbor_cell = &maze[new_row][new_col];
							if (neighbor_cell->cell_val < min_neighbor_val)
							{
								min_neighbor_val = neighbor_cell->cell_val;
							}
							 if (neighbor_cell->cell_val <maze[temp.row_position][temp.col_position].cell_val)
							 {
								 // If any neighbor has a cell_val less than or equal to current_cell's cell_val
								 all_neighbors_greater = 0;
							 }
						}
					}
			    }
			    if(all_neighbors_greater)
			    {
						maze[temp.row_position][temp.col_position].cell_val=min_neighbor_val+1;
						log("value after update : ",maze[temp.row_position][temp.col_position].cell_val);

						log("updated cell row",temp.row_position);
						log("updated cell col",temp.col_position);

					for (int i = 0; i < 4; i++)
					{
						int new_row = maze[temp.row_position][temp.col_position].row_position + directions[i][0];
											int new_col = maze[temp.row_position][temp.col_position].col_position + directions[i][1];
											// Check if the new cell position is within bounds
											if (new_row >= 0 && new_row < MAZE_SIZE && new_col >= 0 && new_col < MAZE_SIZE)
											{
												// Check if there is no wall between current cell and the new cell
												if (walls[i] == no_wall)
												{
													FIFO_enqueue(&queue,maze[new_row][new_col]);
													log("fifo enqueue : ",1);
													log("fifo added cell row : ",maze[new_row][new_col].row_position);
													log("fifo added cell col : ",maze[new_row][new_col].col_position);
												}
											}
					}
			    }
	}
	log("end of update function ",1);
	log("-------------------------------------------- ",1);
}

/*void print_position(cell_t maze [MAZE_SIZE][MAZE_SIZE] , uint16_t size){
	int i ,j;
	for (i = 0; i < size; ++i) {
		for (j = 0; j < size; ++j) {
			API_setText(i,j,maze[i][j].row_position,maze[i][j].col_position);
		}
	}
}
*/
void move_cell(cell_t * current_cell , cell_t * min_neighbor_cell){

	if (current_cell->row_position>min_neighbor_cell->row_position) {

									// check the robot position
									if (head_pos == head_to_up){;}
									else if (head_pos == head_to_right )
									{
										turnLeft();
									}
									else if (head_pos == head_to_left )
									{
										turnRight();
									}
									else
									{
										turnRight();
										turnRight();
									}
									head_pos=head_to_up;
							}
					    	else if(current_cell->row_position<min_neighbor_cell->row_position)
							{
								// check the robot position
									if (head_pos == head_to_down){;}
									else if (head_pos == head_to_right )
									{
										turnRight();


									}
									else if (head_pos == head_to_left )
									{
										turnLeft();


									}
									else
									{
										turnRight();
										turnRight();

									}
									head_pos=head_to_down;

							}
					    	else if(current_cell->col_position>min_neighbor_cell->col_position)
					    	{
								// check the robot position
									if (head_pos == head_to_left){;}
									else if (head_pos == head_to_up )
									{
										turnLeft();
									}
									else if (head_pos == head_to_down )
									{
										turnRight();
									}
									else
									{
										turnRight();
										turnRight();

									}
									head_pos=head_to_left;
					    	}
					    	else if(current_cell->col_position<min_neighbor_cell->col_position)
					    	{
								// check the robot position
									if (head_pos == head_to_right){;}
									else if (head_pos == head_to_up )
									{
										turnRight();
									}
									else if (head_pos == head_to_down )
									{
										turnLeft();
									}
									else
									{
										turnRight();
										turnRight();

									}
									head_pos=head_to_right;

					    	}
							moveForward();


}