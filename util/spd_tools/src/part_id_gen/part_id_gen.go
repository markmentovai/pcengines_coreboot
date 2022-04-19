/* SPDX-License-Identifier: GPL-2.0-or-later */

package main

import (
	"encoding/csv"
	"fmt"
	"io"
	"io/ioutil"
	"log"
	"os"
	"path/filepath"
	"strconv"
	"strings"
)

/*
 * This program allocates DRAM strap IDs for different parts that are being used by the variant.
 *
 * It expects the following inputs:
 *  Name of the SoC platform, e.g. TGL.
 *  Memory technology used by the variant, e.g. lp4x.
 *  Path to Makefile directory. Makefile.inc generated by this program is placed in this
 *  location.
 *  Text file containing a list of memory part names used by the board. Each line in the file
 *  is expected to have one memory part name.
 */
const (
	SPDManifestFileName       = "parts_spd_manifest.generated.txt"
	PlatformsManifestFileName = "platforms_manifest.generated.txt"
	SPDEmptyFileName          = "spd-empty.hex"
	MakefileName              = "Makefile.inc"
	DRAMIdFileName            = "dram_id.generated.txt"
	MaxMemoryId               = 15
)

var supportedPlatforms = [...]string{
	"TGL",
	"ADL",
	"JSL",
	"PCO",
	"CZN",
	"SBR",
}

var supportedMemTechs = [...]string{
	"lp4x",
	"ddr4",
	"lp5",
}

func usage() {
	fmt.Printf("\nUsage: %s <platform> <mem_technology> <makefile_dir> <mem_parts_used_file>\n\n", os.Args[0])
	fmt.Printf("   where,\n")
	fmt.Printf("   platform = SoC platform which the board is based on\n")
	fmt.Printf("              supported platforms: %v\n", supportedPlatforms)
	fmt.Printf("   mem_technology = Memory technology used by the board\n")
	fmt.Printf("                    supported technologies: %v\n", supportedMemTechs)
	fmt.Printf("   makefile_dir = Directory path where generated Makefile.inc should be placed\n")
	fmt.Printf("   mem_parts_used_file = CSV file containing list of memory parts used by the board and optional fixed ids\n\n\n")
}

func checkArgs(platform string, memTech string, makefileDir string, memPartsUsedFile string) error {
	supported := false
	for _, p := range supportedPlatforms {
		if strings.ToUpper(platform) == p {
			supported = true
			break
		}
	}
	if !supported {
		return fmt.Errorf("Platform %s is not supported", platform)
	}

	supported = false
	for _, m := range supportedMemTechs {
		if strings.ToLower(memTech) == m {
			supported = true
			break
		}
	}
	if !supported {
		return fmt.Errorf("Memory technology %s is not supported", memTech)
	}

	if _, err := os.Stat(makefileDir); err != nil {
		return fmt.Errorf("Invalid makefile_dir %s: %v", makefileDir, err)
	}

	if _, err := os.Stat(memPartsUsedFile); err != nil {
		return fmt.Errorf("Invalid mem_parts_used_file %s: %v", memPartsUsedFile, err)
	}

	return nil
}

type mappingType int

const (
	Auto mappingType = iota
	Fixed
	Exclusive
)

type usedPart struct {
	partName    string
	index       int
	mapping     mappingType
	SPDOverride string
}

func readPlatformsManifest(memTech string) (map[string]string, error) {
	manifestFilePath := filepath.Join("spd", strings.ToLower(memTech), PlatformsManifestFileName)
	f, err := os.Open(manifestFilePath)
	if err != nil {
		return nil, err
	}
	defer f.Close()
	r := csv.NewReader(f)
	r.Comment = '#'

	platformToSetMap := make(map[string]string)

	for {
		fields, err := r.Read()

		if err == io.EOF {
			break
		}

		if err != nil {
			return nil, err
		}

		if len(fields) != 2 {
			return nil, fmt.Errorf("Platforms manifest file is incorrectly formatted: %s", manifestFilePath)
		}

		platformToSetMap[fields[0]] = fields[1]
	}

	return platformToSetMap, nil
}

func getSPDDir(platform string, memTech string) (string, error) {
	platformToSetMap, err := readPlatformsManifest(memTech)
	if err != nil {
		return "", err
	}

	setName, ok := platformToSetMap[strings.ToUpper(platform)]
	if !ok {
		return "", fmt.Errorf("Platform %s does not support memory technology %s", platform, memTech)
	}

	return filepath.Join("spd", strings.ToLower(memTech), setName), nil
}

/*
 * Read input file CSV that contains list of memory part names used by the variant
 * and an optional assigned id.
 */
func readParts(memPartsUsedFileName string) ([]usedPart, error) {
	f, err := os.Open(memPartsUsedFileName)
	if err != nil {
		return nil, err
	}
	defer f.Close()
	r := csv.NewReader(f)
	r.FieldsPerRecord = -1 // Allow variable length records
	r.TrimLeadingSpace = true
	r.Comment = '#'

	parts := []usedPart{}

	for {
		fields, err := r.Read()

		if err == io.EOF {
			break
		}

		if err != nil {
			return nil, err
		}

		if len(fields) == 1 {
			parts = append(parts, usedPart{fields[0], -1, Auto, ""})
		} else {
			var mapping = Auto
			var assignedId = -1
			var err error = nil
			var spdOverride string = ""

			// Second column, ID override
			if len(fields) >= 2 {
				if len(fields[1]) >= 2 && fields[1][0] == '*' {
					// Exclusive mapping
					mapping = Exclusive
					assignedId, err = strconv.Atoi(fields[1][1:])
				} else if fields[1] != "" {
					// Fixed mapping
					mapping = Fixed
					assignedId, err = strconv.Atoi(fields[1])
				}
			}

			// Third column, SPD file override
			if len(fields) >= 3 {
				if len(fields[2]) == 0 {
					err = fmt.Errorf("mem_parts_used_file file is incorrectly formatted, SPD file column is empty")
				} else {
					spdOverride = fields[2]
				}
			}

			if err != nil {
				return nil, err
			}

			if assignedId > MaxMemoryId {
				return nil, fmt.Errorf("Out of bounds assigned id %d for part %s", assignedId, fields[0])
			}

			parts = append(parts, usedPart{fields[0], assignedId, mapping, spdOverride})
		}
	}

	return parts, nil
}

/*
 * Read SPD manifest file(CSV) generated by gen_spd program and generate two maps:
 * 1. Part to SPD Map : This maps global memory part name to generated SPD file name
 * 2. SPD to Index Map: This generates a map of deduplicated SPD file names to index assigned to
 *                      that SPD. This function sets the index for all SPDs to -1. This index gets
 *                      updated as part of genPartIdInfo() depending upon the SPDs actually used
 *                      by the variant.
 */
func readSPDManifest(SPDDirName string) (map[string]string, map[string]int, error) {
	f, err := os.Open(filepath.Join(SPDDirName, SPDManifestFileName))
	if err != nil {
		return nil, nil, err
	}
	defer f.Close()
	r := csv.NewReader(f)
	r.Comment = '#'

	partToSPDMap := make(map[string]string)
	SPDToIndexMap := make(map[string]int)

	for {
		fields, err := r.Read()

		if err == io.EOF {
			break
		}

		if err != nil {
			return nil, nil, err
		}

		if len(fields) != 2 {
			return nil, nil, fmt.Errorf("CSV file is incorrectly formatted")
		}

		partToSPDMap[fields[0]] = fields[1]
		SPDToIndexMap[fields[1]] = -1
	}

	return partToSPDMap, SPDToIndexMap, nil
}

/* Print information about memory part used by variant and ID assigned to it. */
func appendPartIdInfo(s *string, partName string, index int) {
	*s += fmt.Sprintf("%-30s %d (%04b)\n", partName, index, int64(index))
}

type partIds struct {
	SPDFileName string
	memParts    string
}

func getFileHeader() string {
	return `# SPDX-License-Identifier: GPL-2.0-or-later
# This is an auto-generated file. Do not edit!!
# Generated by:
` + fmt.Sprintf("# %s\n\n", strings.Join(os.Args[0:], " "))
}

/*
 * For each part used by the variant, check if the SPD (as per the manifest) already has an ID
 * assigned to it. If yes, then add the part name to the list of memory parts supported by the
 * SPD entry. If not, then assign the next ID to the SPD file and add the part name to the
 * list of memory parts supported by the SPD entry.
 *
 * Returns list of partIds that contains spdFileName and supported memory parts for each
 * assigned ID.
 */
func genPartIdInfo(parts []usedPart, partToSPDMap map[string]string, SPDToIndexMap map[string]int, makefileDirName string) ([]partIds, error) {
	partIdList := []partIds{}
	assignedMapping := []mappingType{}
	var s string

	// Assign parts with fixed ids first
	for _, p := range parts {
		if p.index == -1 {
			continue
		}

		if p.partName == "" {
			return nil, fmt.Errorf("Invalid part entry")
		}

		SPDFileName, ok := partToSPDMap[p.partName]
		if !ok {
			return nil, fmt.Errorf("Failed to find part %s in SPD Manifest. Please add the part to global part list and regenerate SPD Manifest", p.partName)
		}

		// Extend partIdList and assignedMapping with empty entries if needed
		for i := len(partIdList) - 1; i < p.index; i++ {
			partIdList = append(partIdList, partIds{})
			assignedMapping = append(assignedMapping, Auto)
		}

		// Only allow parts with the same index if they share the same SPD
		assignedSPD := partIdList[p.index].SPDFileName
		if assignedSPD != "" && assignedSPD != partToSPDMap[p.partName] {
			return nil, fmt.Errorf("ID %d is already assigned to %s, conflicting with %s(%s)", p.index, assignedSPD, p.partName, SPDFileName)
		}

		mapping := assignedMapping[p.index]
		if (mapping == Fixed && p.mapping == Exclusive) || (mapping == Exclusive && p.mapping == Fixed) {
			return nil, fmt.Errorf("Exclusive/non-exclusive conflict in assigning %s to ID %d", p.partName, p.index)
		} else {
			assignedMapping[p.index] = p.mapping
		}

		if partIdList[p.index].memParts == "" {
			partIdList[p.index] = partIds{SPDFileName: SPDFileName, memParts: p.partName}
		} else {
			partIdList[p.index].memParts += ", " + p.partName
		}

		// SPDToIndexMap should point to first assigned index in the used part list
		// Exclusive entries don't update the map because they're not valid for auto assigning
		if SPDToIndexMap[SPDFileName] < 0 && p.mapping != Exclusive {
			SPDToIndexMap[SPDFileName] = p.index
		}
	}

	s += fmt.Sprintf("%-30s %s\n", "DRAM Part Name", "ID to assign")

	// Assign parts with no fixed id
	for _, p := range parts {
		if p.partName == "" {
			return nil, fmt.Errorf("Invalid part entry")
		}

		// Add assigned parts to dram id file in the order they appear
		if p.index != -1 {
			appendPartIdInfo(&s, p.partName, p.index)
			continue
		}

		SPDFileName, ok := partToSPDMap[p.partName]
		if !ok {
			return nil, fmt.Errorf("Failed to find part ", p.partName, " in SPD Manifest. Please add the part to global part list and regenerate SPD Manifest")
		}

		index := SPDToIndexMap[SPDFileName]
		// Only Exclusive mappings don't allow automatic assigning of parts
		if index != -1 && assignedMapping[index] != Exclusive {
			partIdList[index].memParts += ", " + p.partName
			appendPartIdInfo(&s, p.partName, index)
			continue
		}

		// Find first empty index
		for i, partId := range partIdList {
			if partId.SPDFileName == "" {
				index = i
				break
			}
		}

		// Append new entry
		if index == -1 {
			index = len(partIdList)
			if index > MaxMemoryId {
				return nil, fmt.Errorf("Maximum part ID %d exceeded.", MaxMemoryId)
			}
			partIdList = append(partIdList, partIds{})
			assignedMapping = append(assignedMapping, Auto)
		}

		SPDToIndexMap[SPDFileName] = index
		appendPartIdInfo(&s, p.partName, index)
		partIdList[index] = partIds{SPDFileName: SPDFileName, memParts: p.partName}
	}

	fmt.Printf("%s", s)

	s = getFileHeader() + s
	err := ioutil.WriteFile(filepath.Join(makefileDirName, DRAMIdFileName), []byte(s), 0644)

	return partIdList, err
}

/*
 * This function generates Makefile.inc under the variant directory path and adds assigned SPDs
 * to SPD_SOURCES.
 */
func genMakefile(partIdList []partIds, makefileDirName string, SPDDir string, partsDir string) error {
	s := getFileHeader()
	s += fmt.Sprintf("SPD_SOURCES =\n")

	for i := 0; i < len(partIdList); i++ {
		if partIdList[i].SPDFileName == "" {
			s += fmt.Sprintf("SPD_SOURCES += %v ", filepath.Join(SPDDir, SPDEmptyFileName))
			s += fmt.Sprintf("     # ID = %d(0b%04b)\n", i, int64(i))
		} else {
			SPDFileName := partIdList[i].SPDFileName
			path := filepath.Join(partsDir, SPDFileName)

			// Check if the file exists in the directory of the parts file
			if _, err := os.Stat(path); err != nil {
				// File doesn't exist, check spd directory
				path = filepath.Join(SPDDir, SPDFileName)
				if _, err = os.Stat(path); err != nil {
					return fmt.Errorf("Failed to write Makefile, SPD file '%s' doesn't exist", SPDFileName)
				}
			}
			s += fmt.Sprintf("SPD_SOURCES += %v ", path)
			s += fmt.Sprintf("     # ID = %d(0b%04b) ", i, int64(i))
			s += fmt.Sprintf(" Parts = %04s\n", partIdList[i].memParts)
		}
	}

	return ioutil.WriteFile(filepath.Join(makefileDirName, MakefileName), []byte(s), 0644)
}

func main() {
	if len(os.Args) != 5 {
		usage()
		log.Fatal("Incorrect number of arguments")
	}

	platform, memTech, makefileDir, memPartsUsedFile := os.Args[1], os.Args[2], os.Args[3], os.Args[4]

	err := checkArgs(platform, memTech, makefileDir, memPartsUsedFile)
	if err != nil {
		log.Fatal(err)
	}

	SPDDir, err := getSPDDir(platform, memTech)
	if err != nil {
		log.Fatal(err)
	}

	partToSPDMap, SPDToIndexMap, err := readSPDManifest(SPDDir)
	if err != nil {
		log.Fatal(err)
	}

	parts, err := readParts(memPartsUsedFile)
	if err != nil {
		log.Fatal(err)
	}

	// Update our SPD maps with part specific overrides
	for _, p := range parts {
		if p.SPDOverride != "" {
			partToSPDMap[p.partName] = p.SPDOverride
			SPDToIndexMap[p.SPDOverride] = -1
		}
	}

	partIdList, err := genPartIdInfo(parts, partToSPDMap, SPDToIndexMap, makefileDir)
	if err != nil {
		log.Fatal(err)
	}

	if err := genMakefile(partIdList, makefileDir, SPDDir, filepath.Dir(memPartsUsedFile)); err != nil {
		log.Fatal(err)
	}
}
