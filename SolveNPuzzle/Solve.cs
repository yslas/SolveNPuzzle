using System;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;

namespace SolveNPuzzle
{
    public class Solve
    {
        private static List<Node> _openQueue;
        private static List<Node> _closedList;

        private static int _order;

        public async static Task<Node> NPuzzle(Node initNode, int milliseconds)
        {
            CancellationTokenSource cts = new CancellationTokenSource();
            Node node = null;

            try
            {
                cts.CancelAfter(milliseconds);
                node = await AStar(initNode, cts.Token);
            }
            catch (OperationCanceledException)
            {
                throw new Exception("solution not found!");
            }
            finally
            {
                cts.Dispose();
            }

            return node;
        }

        private static Task<Node> AStar(Node initNode, CancellationToken ct)
        {
            return Task.Factory.StartNew(() =>
            {
                ct.ThrowIfCancellationRequested();

                _order = (int)Math.Sqrt(initNode.grid.Length);

                initNode.g = 0;
                initNode.h = (byte)ManhattanDistance(initNode.grid);
                initNode.f = (byte)(initNode.g + initNode.h);

                _openQueue = new List<Node>
                {
                    initNode
                };

                _closedList = new List<Node>();

                while (_openQueue.Count > 0)
                {
                    if (ct.IsCancellationRequested)
                        ct.ThrowIfCancellationRequested();

                    var currentNode = _openQueue[0];
                    _openQueue.RemoveAt(0);
                    _closedList.Add(currentNode);

                    if (ManhattanDistance(currentNode.grid) == 0)
                        return currentNode;

                    foreach (var neighbor in GetNeighbors(currentNode))
                    {
                        Node actualNode = null;

                        if (IsContains(_closedList, neighbor, ref actualNode))
                            continue;
                        if (!IsContains(_openQueue, neighbor, ref actualNode))
                            InsertPriorityInQueue(_openQueue, neighbor);
                        else
                        {
                            if (actualNode.g <= neighbor.g)
                                continue;
                            else
                            {
                                _openQueue.Remove(actualNode);
                                InsertPriorityInQueue(_openQueue, neighbor);
                            }
                        }
                    }
                }

                return null;
            }, ct);
        }

        private static bool IsContains(List<Node> list, Node node, ref Node actualNode)
        {
            foreach (var item in list)
            {
                if (IsEquals(item.grid, node.grid))
                {
                    actualNode = item;
                    return true;
                }
            }
            return false;
        }

        private static void InsertPriorityInQueue(List<Node> queue, Node node)
        {
            int i = 0;

            while (i < queue.Count && node.f > queue[i].f)
                i++;
            if (i >= queue.Count)
                queue.Add(node);
            else
                queue.Insert(i, node);
        }

        private static int ManhattanDistance(byte[,] currentGrid)
        {
            int heuristic = 0;

            for (int i = 0; i < _order; i++)
            {
                for (int j = 0; j < _order; j++)
                {
                    if (currentGrid[i, j] != 0)
                    {
                        int posI = (currentGrid[i, j] - 1) / _order;
                        int posJ = (currentGrid[i, j] - 1) - (posI * _order);
                        heuristic += Math.Abs(i - posI) + Math.Abs(j - posJ);
                    }
                }
            }
            return heuristic;
        }

        private static List<Node> GetNeighbors(Node node)
        {
            var nodes = new List<Node>();

            if (node.x > 0)
                nodes.Add(GetNeighbor(node, (node.x - 1), node.y));

            if (node.x < _order - 1)
                nodes.Add(GetNeighbor(node, (node.x + 1), node.y));

            if (node.y > 0)
                nodes.Add(GetNeighbor(node, node.x, (node.y - 1)));

            if (node.y < _order - 1)
                nodes.Add(GetNeighbor(node, node.x, (node.y + 1)));

            return nodes;
        }

        private static Node GetNeighbor(Node node, int emptyX, int emptyY)
        {
            byte[,] newGrid = GetNewGrid(node.grid, emptyX, emptyY);
            var newNode = new Node(node, newGrid, (byte)emptyX, (byte)emptyY)
            {
                g = (byte)(node.g + 1)
            };

            newNode.h = (byte)ManhattanDistance(newNode.grid);
            newNode.f = (byte)(newNode.g + newNode.h);
            newNode.parent = node;

            return newNode;
        }

        private static byte[,] GetNewGrid(byte[,] actualGrid, int actualEmptyX, int actualEmptyY)
        {
            byte[,] grid = new byte[_order, _order];
            int emptyX = 0, emptyY = 0;

            for (int i = 0; i < _order; i++)
            {
                for (int j = 0; j < _order; j++)
                {
                    grid[i, j] = actualGrid[i, j];
                    if (grid[i, j] == 0)
                    {
                        emptyX = i;
                        emptyY = j;
                    }
                }
            }

            byte aux = grid[actualEmptyX, actualEmptyY];
            grid[actualEmptyX, actualEmptyY] = grid[emptyX, emptyY];
            grid[emptyX, emptyY] = aux;

            return grid;
        }

        private static bool IsEquals(byte[,] currentGrid, byte[,] goalGrid)
        {
            for (int i = 0; i < _order; i++)
            {
                for (int j = 0; j < _order; j++)
                {
                    if (currentGrid[i, j] != goalGrid[i, j])
                        return false;
                }
            }
            return true;
        }
    }
}