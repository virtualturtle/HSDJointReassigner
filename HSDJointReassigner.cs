using System;
using System.Collections.Generic;
using System.IO;
using HSDRaw;
using HSDRaw.Common;
using HSDRaw.AirRide.Vc;

namespace JointReassigner
{
    class Program
    {
        static int Main(string[] args)
        {
            if (args.Length < 6)
            {
                Console.WriteLine("Usage:");
                Console.WriteLine("  JointReassigner.exe <input.dat> <output.dat> --from-joint <J#> --object <O#> --to-joint <J#>");
                Console.WriteLine("Example:");
                Console.WriteLine("  JointReassigner.exe vehicle.dat vehicle_modified.dat --from-joint 0 --object 2 --to-joint 6");
                return 1;
            }

            string inputPath = args[0];
            string outputPath = args[1];

            // naive arg parse (expects exact positions for flags)
            int fromJoint = -1;
            int objectIndex = -1;
            int toJoint = -1;

            for (int i = 2; i < args.Length; i++)
            {
                switch (args[i])
                {
                    case "--from-joint":
                        if (++i < args.Length && int.TryParse(args[i], out var fj)) fromJoint = fj;
                        else { Console.WriteLine("Invalid or missing value for --from-joint"); return 1; }
                        break;
                    case "--object":
                        if (++i < args.Length && int.TryParse(args[i], out var oi)) objectIndex = oi;
                        else { Console.WriteLine("Invalid or missing value for --object"); return 1; }
                        break;
                    case "--to-joint":
                        if (++i < args.Length && int.TryParse(args[i], out var tj)) toJoint = tj;
                        else { Console.WriteLine("Invalid or missing value for --to-joint"); return 1; }
                        break;
                    default:
                        Console.WriteLine($"Unknown argument: {args[i]}");
                        return 1;
                }
            }

            if (fromJoint < 0 || objectIndex < 0 || toJoint < 0)
            {
                Console.WriteLine("All of --from-joint, --object and --to-joint must be specified and >= 0");
                return 1;
            }

            if (!File.Exists(inputPath))
            {
                Console.WriteLine($"Input file not found: {inputPath}");
                return 1;
            }

            try
            {
                var file = new HSDRawFile(inputPath);

                if (file.Roots.Count == 0)
                {
                    Console.WriteLine("No root nodes found in file.");
                    return 1;
                }

                if (file.Roots[0].Data is not KAR_vcDataStar machineData)
                {
                    Console.WriteLine("Root data is not KAR_vcDataStar.");
                    return 1;
                }

                var modelRoot = machineData.ModelData?.MainModelRoot;
                if (modelRoot == null)
                {
                    Console.WriteLine("No MainModelRoot found in model data.");
                    return 1;
                }

                // Build joint index map (depth-first)
                var jointList = new List<HSD_JOBJ>();
                BuildJointList(modelRoot, jointList);

                // Validate joint indices
                if (fromJoint >= jointList.Count)
                {
                    Console.WriteLine($"from-joint index {fromJoint} out of range (0..{jointList.Count - 1})");
                    return 1;
                }
                if (toJoint >= jointList.Count)
                {
                    Console.WriteLine($"to-joint index {toJoint} out of range (0..{jointList.Count - 1})");
                    return 1;
                }

                var fromJobj = jointList[fromJoint];
                var toJobj = jointList[toJoint];

                Console.WriteLine($"Moving object O{objectIndex} from J{fromJoint} -> J{toJoint}.");

                // Find the DOBJ and remove it from fromJobj's DOBJ linked list
                var removedDobj = RemoveDobjAtIndex(fromJobj, objectIndex);
                if (removedDobj == null)
                {
                    Console.WriteLine($"Object index O{objectIndex} not found under J{fromJoint}.");
                    return 1;
                }

                // Append removedDobj to destination joint's DOBJ list
                AppendDobjToJoint(toJobj, removedDobj, out int newObjectIndex);

                Console.WriteLine($"Successfully moved. New object index under J{toJoint} is O{newObjectIndex}.");

                // Save file
                try
                {
                    // Try to use HSDRawFile's Save method. If your HSDRaw version uses a different method name,
                    // change file.Save(outputPath) to the correct call (e.g., file.SaveAs(outputPath)).
                    file.Save(outputPath);
                    Console.WriteLine($"Saved modified file to: {outputPath}");
                }
                catch (MissingMethodException)
                {
                    // If Save() doesn't exist, attempt Write to stream as fallback
                    Console.WriteLine("Warning: file.Save(path) not available in this HSDRaw version. Attempting fallback write...");
                    using var fs = new FileStream(outputPath, FileMode.Create, FileAccess.Write);
                    file.Save(fs); // some builds expose Save(Stream) – try this
                    Console.WriteLine($"Saved modified file to: {outputPath} (fallback)");
                }

                return 0;
            }
            catch (Exception ex)
            {
                Console.WriteLine("Error: " + ex.Message);
                Console.WriteLine(ex.ToString());
                return 1;
            }
        }

        // Depth-first enumeration of all HSD_JOBJ nodes
        private static void BuildJointList(HSD_JOBJ root, List<HSD_JOBJ> list)
        {
            if (root == null) return;
            list.Add(root);
            foreach (var child in root.Children)
                BuildJointList(child, list);
        }

        // Remove the DOBJ at the given zero-based index from jobj.Dobj linked list.
        // Returns the removed DOBJ (with Next set to null) or null if not found.
        private static HSD_DOBJ RemoveDobjAtIndex(HSD_JOBJ jobj, int index)
        {
            if (jobj == null) return null;

            var head = jobj.Dobj;
            if (head == null) return null;

            HSD_DOBJ prev = null;
            HSD_DOBJ cur = head;
            int i = 0;
            while (cur != null && i < index)
            {
                prev = cur;
                cur = cur.Next;
                i++;
            }

            if (cur == null) return null; // not found

            // Remove cur from list
            if (prev == null)
            {
                // removing head
                jobj.Dobj = cur.Next;
            }
            else
            {
                prev.Next = cur.Next;
            }

            // isolate removed node
            cur.Next = null;
            return cur;
        }

        // Append a DOBJ to the end of the jobj.Dobj linked list.
        // Returns new index via out parameter.
        private static void AppendDobjToJoint(HSD_JOBJ jobj, HSD_DOBJ dobjToAdd, out int newIndex)
        {
            newIndex = 0;
            if (jobj == null || dobjToAdd == null) return;

            var head = jobj.Dobj;
            if (head == null)
            {
                jobj.Dobj = dobjToAdd;
                dobjToAdd.Next = null;
                newIndex = 0;
                return;
            }

            int i = 0;
            var cur = head;
            while (cur.Next != null)
            {
                cur = cur.Next;
                i++;
            }

            // cur is last node; append after it
            cur.Next = dobjToAdd;
            dobjToAdd.Next = null;
            newIndex = i + 1;
        }
    }
}
