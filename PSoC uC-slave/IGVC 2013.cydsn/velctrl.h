/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/

void UpdateVelCommands(int lx, int az);
void UpdateAngularZ(int az);
void UpdateLinearX(int lx);
int GetV(void);
int GetW(void);
void SetAccelDivisor(int in);
void RunVelocityControl(void);
void ClearVelocityControl(void);
void UpdateVelocity(void);
void InitializeVelocityControl(void);

//[] END OF FILE
