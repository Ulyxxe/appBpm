//-----------------------------------------------------------------------------
//  Gnx_Projet_ECG.h
// 
//-----------------------------------------------------------------------------

#define PIN_LED_R     PF_1
#define PIN_LED_G     PF_2
#define PIN_LED_B     PF_3
#define PIN_POTAR     PE_3
#define PIN_BOUTON    PC_6


//------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------
void	Sensor_Init(void);
short	Sensor_Measure(void);


//------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------
void    AOLED_InitScreen(void);
void    AOLED_AffiLogoIsep(void);
void    AOLED_FillScreen(   char  value);
void    AOLED_InvertDisplay(short invert);
void    AOLED_DisplayImage( char  *pBuff);
void    AOLED_DisplayCarac( int x, int y, char car);
void    AOLED_DisplayTexte( int x, int y, char* texte);
