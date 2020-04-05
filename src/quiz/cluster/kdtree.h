/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
    std::vector<float> point;
    int id;
    Node* left;
    Node* right;

    Node(std::vector<float> arr, int setId)
        :	point(arr), id(setId), left(NULL), right(NULL)
    {}
};

struct KdTree
{
    Node* root;

    KdTree()
        : root(NULL)
    {}
    //    void insert_data(Node* node_ptr, uint depth, std::vector<float> point, int id)
    //    {

    //        if (node_ptr == NULL) // if
    //        {
    //            // Now node_ptr is pointing to node filled with point and id
    //            // and it has 2 child roots which are pointing to NULL.
    //            node_ptr = new Node(point, id);
    //        }
    //        else
    //        {
    //            uint current_dim = depth%2;

    //            if (point.at(current_dim) < node_ptr->point.at(current_dim))
    //            {
    //                insert_data(node_ptr->left, depth+1, point, id);
    //            }
    //            else
    //            {
    //                insert_data(node_ptr->right, depth+1, point, id);
    //            }

    //        }

    //    }

    //    void insert(std::vector<float> point, int id)
    //    {
    //        // root is a pointer to Node
    //        insert_data(root, 0, point, id);
    //    }
    void insert_data(Node** root_ptr, uint depth, std::vector<float> point, int id)
    {

        if (*root_ptr == NULL) // if
        {
            // Now node_ptr is pointing to node filled with point and id
            // and it has 2 child roots which are pointing to NULL.
            *root_ptr = new Node(point, id);
        }
        else
        {
            uint current_dim = depth%2;

            if (point.at(current_dim) < (*root_ptr)->point.at(current_dim))
            {
                insert_data(&((*root_ptr)->left), depth+1, point, id);
            }
            else
            {
                insert_data(&((*root_ptr)->right), depth+1, point, id);
            }

        }

    }

    void insert(std::vector<float> point, int id)
    {
        // root is a pointer to Node
        insert_data(&root, 0, point, id);
    }

    //    void insert(std::vector<float> point, int id)
    //    {


    //        std::cout << "point is : " << point[0] << " " << point[1] << std::endl;
    //        std::cout << "Root is " << root << std::endl;

    //        int depth = 0;
    //        if (root == NULL)
    //        {
    //            root = new Node(point, id);
    //        }
    //        while(root!=NULL)
    //        {
    //            if(depth%2 == 0)          // depth = 0,2,4,6 comparisons to be done on x
    //            {
    //                if(point.at(0) < (root->point).at(0))
    //                {
    //                    // insert the node to left children
    //                    //                    root = root->left;
    //                    root->left = new Node(point,id);
    //                    //                    depth++;

    //                }
    //                else
    //                {
    //                    // insert the node to right children
    //                    //                    root = root->right;
    //                    root->right = new Node(point, id);

    //                }
    //            }
    //            else                    // depth = 1,3,5,7 comparisons to be done on y
    //            {
    //                if(point.at(1) < (root->point).at(1))
    //                {
    //                    // insert the node to left children
    //                    //                    root = root->left;
    //                    root->left = new Node(point,id);
    //                }
    //                else
    //                {
    //                    // insert the node to right children
    //                    //                    root = root->right;
    //                    root->right = new Node(point, id);
    //                }
    //            }
    //        }
    //        //        else if(depth%2 !=0)
    //        std::cout << "root now is : " << (root->point).at(0) << "\t" << (root->point)[1] << std::endl;
    //        std::cout << "root children are " << root->left << "\t" << root->right << std::endl;


    //        //            if(id==0 || depth%2==0)
    //        //            {

    //        //            }
    //        //            else
    //        //            {
    //        //            }

    //        // TODO: Fill in this function to insert a new point into the tree
    //        // the function should create a new node and place correctly with in the root

    //    }

    void searchHelper(Node* root_ptr, uint depth, std::vector<float> target, float distanceTol, std::vector<int>& ids)
    {


        if(root_ptr !=NULL)
        {
            uint current_dim = depth%2;

            if( (root_ptr->point[0] >= (target[0]-distanceTol)) && (root_ptr->point[0] <= (target[0]+distanceTol) )
                && (root_ptr->point[1] >= (target[1]-distanceTol)) && (root_ptr->point[1] <= (target[1]+distanceTol))   )
            {
                // the node is inside the box, hence calculate the distance
                float distance = std::sqrt(std::pow((root_ptr->point[0]-target[0]),2)+std::pow((root_ptr->point[1]-target[1]),2));
                std::cout << "the distance is " << distance << std::endl;
                if (distance <= distanceTol)
                {
                    ids.push_back(root_ptr->id);
                }
            }


            // Checking for boundaries
            if(target.at(current_dim) - distanceTol < (root_ptr->point).at(current_dim) )
            {
                searchHelper(root_ptr->left, depth+1, target, distanceTol, ids);
            }
            if (target.at(current_dim) + distanceTol > (root_ptr->point).at(current_dim))
            {
                searchHelper(root_ptr->right, depth+1, target, distanceTol, ids);
            }

        }

    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(std::vector<float> target, float distanceTol)
    {
        std::vector<int> ids;

        searchHelper(root, 0, target, distanceTol, ids);

        return ids;
    }


};




