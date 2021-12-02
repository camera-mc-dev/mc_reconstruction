import os,sys,shutil

class Chapter:
	def __init__(self, index, title, cmarkFiles):
		self.index = index
		self.title = title
		self.files = cmarkFiles

class Book:
	def __init__(self):
		self.chapters = {}
		self.title        = None
		self.author       = None
		self.date         = None
		self.coverImg     = None
		self.style        = None
		self.breakstyle   = "----"
		self.outputTypes  = ["html"]
		
	def MakeChapter(self, index, title, cmarkFiles):
		ch = Chapter(index, title, cmarkFiles)
		self.chapters[index] = ch
		
	def Compile(self):
		for ot in self.outputTypes:
			if ot == "html":
				self.CompileHTML()
			if ot == "pdf":
				self.CompilePdf()
			if ot == "epub":
				self.CompileEpub()
			if ot == "word":
				self.CompileWord()
				
	
	def CompileHTML(self):
		# make the output directory.
		if( os.path.exists("html") ):
			shutil.rmtree("html")
		os.mkdir("html")
		
		chapKeys = sorted( self.chapters.keys() )
		
		# make html cover page
		coverFile = open(".cover", 'w')
		coverFile.write('<div style="text-align: center">\n')
		if( self.coverImg != None ):
			imgf = "cover." + self.coverImg[ self.coverImg.rfind(".")+1: ]
			cmd = "cp %s html/%s"%(self.coverImg, imgf)
			print( cmd )
			os.system( cmd )
			coverFile.write("![cover](%s){ width=90%% }\\ \n\n"%imgf)
		coverFile.write("# %s\n\n"%self.title)
		coverFile.write("%s\n\n"%self.author)
		coverFile.write("%04d-%02d-%02d\n\n"%(self.date[0], self.date[1], self.date[2]) )
		for k in chapKeys:
			coverFile.write("[%s](%04d.html) \n\n"%( self.chapters[k].title, k ) )
		coverFile.write("</div>\n\n")
		coverFile.close()
		cmd = "pandoc  -s -c %s .cover --metadata pagetitle=%s -o html/index.html"%(self.style, "cover")
		os.system(cmd)

		# copy imgs to html directory
		os.system("cp -r imgs html/")
		
		# run pandoc to compile all chapter source files into the output directory
		for kc in range( len(chapKeys) ):
			print( kc, chapKeys[kc] )
			k = chapKeys[kc]
			
			# create chapter title
			titleFile = open(".title", 'w')
			titleFile.write("[contents](index.html)\n")
			if( kc > 0 ):
				titleFile.write("[previous chapter](%04d.html)\n\n"%(chapKeys[kc-1]) )
			titleFile.write("\n")
			titleFile.write("# %s\n"%self.chapters[k].title )
			titleFile.close()
			cmd = 'pandoc -s --mathjax --metadata pagetitle="%s" .title'%self.chapters[k].title
			
			# add all the contents files
			if( self.style != None ):
				cmd = cmd + " -c %s"%self.style
				os.system( "cp %s html/"%self.style )
			for f in self.chapters[k].files:
				cmd = cmd + " %s"%f
			
			# now add the link to the next chapter, and anything else we want as a footer.
			footFile = open(".foot", 'w')
			footFile.write("[contents](index.html)\n")
			if( kc+1 < len(chapKeys) ):
				footFile.write("[next chapter](%04d.html)\n\n"%(chapKeys[kc+1] ) )
			footFile.close()
			cmd = cmd + " --highlight=kate .foot -o html/%04d.html"%(k)
			print( cmd )
			os.system(cmd)

		cmd = "rm .cover .title* .foot"
		os.system(cmd)
			
		
		
	def CompilePdf(self):
		pass
	
	def CompileEpub(self):
		print( "epub..." )
		# So, one of the reasons to use Pandoc is that it should make producing epubs
		# pretty easy, which is good for reading on my phone.
		
		# first off, we need a metadata file in YAML format.
		mf = open(".epubmeta.yml", 'w')
		mf.write("---\n")
		mf.write("title:\n- type: main\n  text: %s\n"%self.title)
		mf.write("creator:\n- role: author\n  text: %s\n"%self.author)
		mf.write("cover-image: %s\n"%self.coverImg )
		#mf.write("stylesheet: %s\n"%self.style )
		mf.write("...")
		mf.close()
		
		# and with that done, we just do the compile...
		chapKeys = sorted( self.chapters.keys() )
		cmd = "pandoc -o %s.epub .epubmeta.yml "%( self.title.replace(" ", "_") )
		for k in chapKeys:
			tfn = ".title-" + str(k)
			tf  = open( tfn, 'w')
			tf.write("# %s\n\n"%self.chapters[k].title)
			tf.close()
			
			cmd = "%s %s "%(cmd, tfn)
			for f in self.chapters[k].files:
				cmd	= "%s %s "%(cmd, f)
		
		os.system(cmd)

	def CompileWord(self):
		print( "Word..." )
		# annoyingly, many of the agency submissions seem to want to work with fucktard Word.
		# That is not entirely surprising to be honest, and is one of the reasons we
		# switched to commonmark and Pandoc from our own format.
		
		# make the output directory.
		if( os.path.exists("word") ):
			shutil.rmtree("word")
		os.mkdir("word")

		# make an intro / title section.
		coverFile = open(".cover", 'w')
	
		coverFile.write("# %s\n\n"%self.title)
		coverFile.write("%s\n\n"%self.author)
		coverFile.write("%04d-%02d-%02d\n\n"%(self.date[0], self.date[1], self.date[2]) )
		coverFile.close()

		# now we pass all the various chapter files into pandoc and get it to spit out
		# a word file, hoping the process that it actually manages...
		# overall, that's pretty good, but the scene-breaks are a bit annoying.
		# still, much better than my trying to do it! Having said that, I would guess
		# word can just open an html file if needed...
		chapKeys = sorted( self.chapters.keys() )
		cmd = "pandoc -c %s .cover "%(self.style)
		for k in chapKeys:
			tfn = ".title-" + str(k)
			tf  = open( tfn, 'w')
			tf.write("# %s\n\n"%self.chapters[k].title)
			tf.close()

			cmd = "%s %s "%(cmd, tfn)
			for f in self.chapters[k].files:
				cmd	= "%s %s "%(cmd, f)

		cmdWord = "%s -o word/%s.doc"%(cmd, self.title.replace(" ", "_") )
		cmdHtml = "%s -o word/%s.html"%(cmd, self.title.replace(" ", "_" ) )
		
		os.system(cmdWord)
		os.system(cmdHtml)


