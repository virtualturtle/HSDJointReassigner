using System;
using System.Collections.Generic;
using System.IO;
using HSDRaw;
using HSDRaw.Common;
using HSDRaw.AirRide.Vc;

namespace HSDJointReassigner
{
    internal class Program
    {
        private static void Main(string[] args)
        {
            if (args.Length == 0)
            {
                Console.WriteLine("Drag and drop a .dat file onto this program to run.");
                return;
            }

            string filePath = args[0];
            if (!File.Exists(filePath))
            {
                Console.WriteLine($"File not found: {filePath}");
                return;
            }

            var file = new HSDRawFile(filePath);

            if (file.Roots.Count == 0 || file.Roots[0].Data is not KAR_vcDataStar machineData)
            {
                Console.WriteLine("Invalid or empty .dat file.");
                return;
            }

            var rootJOBJ = machineData.ModelData.MainModelRoot;

            // Build joint → DOBJ mapping
            var jointDobjs = new Dictionary<int, List<HSD_DOBJ>>();
            BuildJointDobjMap(rootJOBJ, jointDobjs);

            // Display current assignments
            Console.WriteLine("Current DOBJ joint assignments:");
            DisplayJointDobjs(jointDobjs);

            // Ask user which object to reassign
            while (true)
            {
                Console.WriteLine("Enter object index to reassign (or 'done' to finish):");
                string? input = Console.ReadLine();
                if (input == null || input.Trim().ToLower() == "done")
                    break;

                if (!int.TryParse(input, out int objIndex) || objIndex < 0)
                {
                    Console.WriteLine("Invalid index.");
                    continue;
                }

                // Find object and its current joint
                int foundJoint = -1;
                HSD_DOBJ? selectedDobj = null;
                foreach (var kvp in jointDobjs)
                {
                    if (objIndex < kvp.Value.Count)
                    {
                        selectedDobj = kvp.Value[objIndex];
                        foundJoint = kvp.Key;
                        break;
                    }
                    objIndex -= kvp.Value.Count;
                }

                if (selectedDobj == null)
                {
                    Console.WriteLine("Object index out of range.");
                    continue;
                }

                Console.WriteLine($"Selected DOBJ on Joint {foundJoint}. Enter new joint index:");
                string? jointInput = Console.ReadLine();
                if (!int.TryParse(jointInput, out int newJoint) || newJoint < 0)
                {
                    Console.WriteLine("Invalid joint index.");
                    continue;
                }

                // Remove from old joint list
                jointDobjs[foundJoint].Remove(selectedDobj);

                // Add to new joint list, create list if needed
                if (!jointDobjs.ContainsKey(newJoint))
                    jointDobjs[newJoint] = new List<HSD_DOBJ>();
                jointDobjs[newJoint].Add(selectedDobj);

                Console.WriteLine($"Moved DOBJ from Joint {foundJoint} → Joint {newJoint}");
                DisplayJointDobjs(jointDobjs);
            }

            // Apply changes back to HSD_JOBJ.Dobj linked lists
            ApplyJointDobjMap(rootJOBJ, jointDobjs);

            // Save edited file
            string outPath = Path.Combine(Path.GetDirectoryName(filePath)!,
                Path.GetFileNameWithoutExtension(filePath) + "_edited.dat");
            file.Save(outPath);
            Console.WriteLine($"Saved edited file to {outPath}");
        }

        // Build dictionary of joint index → list of DOBJ
        private static void BuildJointDobjMap(HSD_JOBJ jobj, Dictionary<int, List<HSD_DOBJ>> jointDobjs)
        {
            int jointCounter = 0;
            void Traverse(HSD_JOBJ j, ref int counter)
            {
                if (j == null) return;

                var dobjList = new List<HSD_DOBJ>();
                var current = j.Dobj;
                while (current != null)
                {
                    dobjList.Add(current);
                    current = current.Next;
                }
                jointDobjs[counter++] = dobjList;

                foreach (var child in j.Children)
                    Traverse(child, ref counter);
            }

            Traverse(jobj, ref jointCounter);
        }

        // Display all DOBJ assignments
        private static void DisplayJointDobjs(Dictionary<int, List<HSD_DOBJ>> jointDobjs)
        {
            int displayIndex = 0;
            foreach (var kvp in jointDobjs)
            {
                int joint = kvp.Key;
                for (int i = 0; i < kvp.Value.Count; i++)
                {
                    Console.WriteLine($"{displayIndex}: DOBJ on Joint {joint}, ID {i}");
                    displayIndex++;
                }
            }
        }

        // Apply dictionary changes back to the HSD_JOBJ hierarchy
        private static void ApplyJointDobjMap(HSD_JOBJ rootJOBJ, Dictionary<int, List<HSD_DOBJ>> jointDobjs)
        {
            int jointCounter = 0;

            void Traverse(HSD_JOBJ j)
            {
                if (j == null) return;

                if (jointDobjs.ContainsKey(jointCounter))
                {
                    var dobjs = jointDobjs[jointCounter];
                    j.Dobj = dobjs.Count > 0 ? dobjs[0] : null;

                    // Rebuild linked list
                    for (int i = 0; i < dobjs.Count; i++)
                    {
                        dobjs[i].Next = (i + 1 < dobjs.Count) ? dobjs[i + 1] : null;
                    }
                }

                jointCounter++;
                foreach (var child in j.Children)
                    Traverse(child);
            }

            Traverse(rootJOBJ);
        }
    }
}
