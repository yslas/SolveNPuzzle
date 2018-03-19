namespace SolveNPuzzle
{
    public class Node
    {
        //current matrix grid
        public byte[,] grid;

        //g = cust, h = heuristic, f = score(g + h)
        public byte g, h, f;

        //position empty
        public byte x, y;

        //parent
        public Node parent;

        public Node(Node parent, byte[,] grid, byte x, byte y)
        {
            this.parent = parent;
            this.grid = grid;
            this.x = x;
            this.y = y;
        }
    }
}