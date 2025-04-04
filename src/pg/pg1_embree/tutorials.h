#ifndef TUTORIALS_H_
#define TUTORIALS_H_

void error_handler( void * user_ptr, const RTCError code, const char * str = nullptr );

int tutorial_1( const char * config = "threads=0,verbose=3" );
int tutorial_2();
int tutorial_3( const std::string file_name, const char * config = "threads=0,verbose=0" );
int tutorial_4(const char* cubemap_file_names[6], const char* config = "threads=0,verbose=0");

#endif
