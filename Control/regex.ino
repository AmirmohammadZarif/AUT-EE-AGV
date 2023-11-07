#include <Regexp.h>

void Setup(){

Serial.begin(115200);


}
// called for each match
void match_callback  (const char * match,          // matching string (not null-terminated)
                      const unsigned int length,   // length of matching string
                      const MatchState & ms)      // MatchState in use (to get captures)
{
char cap [10];   
  
  Serial.print ("Matched: ");
  Serial.write ((byte *) match, length);
  Serial.println ();
  
  for (byte i = 0; i < ms.level; i++)
    {
    Serial.print ("Capture "); 
    Serial.print (i, DEC);
    Serial.print (" = ");
    ms.GetCapture (cap, i);
    Serial.println (cap); 
    }  // end of for each capture

}  // end of match_callback 

void Loop(){

    if(Serial.available() > 0){
        String recieved = Serial.read(); 
        inData += recieved;
        if (recieved == '\n') {
            MatchState ms;
            ms.Target(inData);

            char result = ms.GlobalMatch("/[a-zA-Z]+-(\d+)+ (\d+)+ (\d+)+/gm", match_callback);
            if (result == REGEXP_MATCHED)
            {

            }
            else if (result == REGEXP_NOMATCH)
            {
            }
            else
            {
            }
            // char parameter = inData[0];
            // String Value = inData.substring(1, inData.length());


         
        }

    }
}