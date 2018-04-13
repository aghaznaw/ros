import re 
import os 

with open ('gazz.txt') as fp:  #opens as file object called fp the file name gazz.txt

	line = fp.readlines() #read one line
	cnt = 1
	while line:
		print ("{}:{}".format (cnt, line.strip()))
		line = fp.readline()
		cnt += 1		#loop to iterate through file reading one line at a time, print Line no: at start of each new line

class testClass:
	def main():
		pattern = re.compile("<model name='bookshelf'>")
		static = re.compile("<static>")
		coordinates = re.compile(".*>\s*([\d.\s-]+)(0)") ## After > select 6 coordinates each separated by a space

		for i, line in enumerate(open('test.sdf')):
			for match in re.finditer(pattern, line):
				print ('Found on line %s: %s' % (i+1, match.groups())
				## IF FOUND INCREMENT ONE LINE AND RETURN STATIC OR NOT AND COORDINATES
				##offset += len(line) # This offset syntax rejected in python3
				##re.findall (static)

if __name__ == '__main__':