#ifndef ROSNEURO_WRITER_HPP
#define ROSNEURO_WRITER_HPP


namespace rosneuro {


class Writer {

	public:
		Writer(void);
		virtual ~Writer(void);

		virtual bool Setup(void) = 0;
		virtual bool Open(const std::string& filename) = 0;
		virtual bool Close(void) = 0;
		virtual int Write(int nswrite) = 0;

		//virtual bool AddEvent(void);

};


}


#endif
