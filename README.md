# LATINAMERICA2022-PLL-CODES






<div class="alert alert-block alert-info">
    <img src="https://www.uerj.br/wp-content/uploads/2019/07/logo-70anos-site.svg" alt="Trulli" width="290" align="left">
    <center style="width: 80%;">
        <div> <b>Authors:</b></div>
        <div>Luís Fernando Corrêa Monteiro</div>
        <div>Cleiton Magalhães Freitas</div>
        <div>Michel Pompeu Tcheou</div>
        <div>Dayane Mendonça Lessa</div>
        <div> <b>Date:</b> 11/07/2022</div>
    </center>
</div>



<br>
<br>


Repository containing the codes used in the article "Improvements on E-PLL to Mitigate Transient Low-Frequency Oscillations" published in the IEEE Transaction on Latin America.

The repository contain the following folders:


## Octave Folder

This folder contain the [Octave](https://octave.org/index) files used for simulating the PLLs presented in the article. The file <code>EPLL_20210220_a.m</code> presents the code for the simulation of the tradition E-PLL, whereas the  <code>SOGI_EPLL_20210218_a.m</code> presents the simulating codes for the proposed PLL.


## CSS Folder

Folder contains the codes used for implementing the proposed PLL in the development kit [LAUNCHXL-F28379D](https://www.ti.com/tool/LAUNCHXL-F28379D). It was considered for this task the IDE [Code Composer Studio 11.1](https://www.ti.com/tool/download/CCSTUDIO/11.1.0.00011).

Within the folder you may find the files <code>coreFunctions.c</code> and <code>coreFunctions.h</code> with the coding of the functions used in the project, among them the function <code>SOGI_EPLL_mod</code> which implements the proposed PLL. The <code>main</code> function, which is implemented in the <code>adc_ex2_soc_epwm.c</code> file, sets all the configurations such as PWM for analog exporting and interruptions. It is also within <code>adc_ex2_soc_epwm.c</code> file that the interruption <code>adcA1ISR</code>, that calls <code>SOGI_EPLL_mod</code>, is defined.   
