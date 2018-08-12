org 100h 

PORTA equ 0x00
PORTB equ 0x40
PORTC equ 0x80
PORT_CON equ 0xC0    

TIMER0 equ  0x200
TIMER_CONTROL equ 0x2C0  

jmp start_code

KEYPAD_DATA db 1,2,3,4,5,6,7,8,9,0xA,0x0,0xB    
msg1 db 'Digital       Lock',0x10,'  HR.Mehrabian ',0
msg2 db '     Hello!     ',0x10,'  From 8086 !!  ',0  
msg3 db 'Enter Password :',0x10,'     ',0
msg4 db '     ',0x0 
msg5 db 'Enter New Pass :',0x10,'     ',0 
msg6 db '  Wrong Pass !! ',0x10,'Invalid Password',0
msg7 db '  Valid Pass !! ',0x10,'NewPass/OpenDoor',0  
msg8 db ' Openning Door!! ',0x10,' Please Wait.. ',0
msg9 db 'Enter New Pass : ',0x10,'     ',0 
vara dw 0
varb dw 0
varc db 0
vard db 0
password db 0,0,0,0,0,0,0,0
tpassword db 0,0,0,0,0,0,0,0
start_code:
mov al,0x88
out PORT_CON,al   


  
call lcd_init
lea bx,msg1
call lcd_puts
mov bx,0xFF
call delay
call delay
call lcd_cls
lea bx,msg2
call lcd_puts
mov bx,0xFF
call delay  
call delay

again: 
call lcd_cls
lea bx,msg3
call lcd_puts
call get_pass
call check_pass
cmp ax,0
jnz valid_pass 
call lcd_home
lea bx,msg6
call lcd_puts
mov bx,0xFF
mov al,0x36 
mov dx, TIMER_CONTROL
out dx,al  
mov al,0x10 
mov dx, TIMER0
out dx,al 
mov al,0x00
out dx,al
call delay  
mov al,0x11 
mov dx, TIMER0
out dx,al 
mov al,0x00
out dx,al
call delay 
mov al,0x12 
mov dx, TIMER0
out dx,al 
mov al,0x00
out dx,al
call delay
mov al,0x13 
mov dx, TIMER0
out dx,al 
mov al,0x00
out dx,al
call delay
mov al,0x14 
mov dx, TIMER0
out dx,al 
mov al,0x00
out dx,al
call delay
mov al,0x15 
mov dx, TIMER0
out dx,al 
mov al,0x00
out dx,al
call delay
mov al,0x16 
mov dx, TIMER0
out dx,al 
mov al,0x00
out dx,al
call delay
mov al,0x17 
mov dx, TIMER0
out dx,al 
mov al,0x00
out dx,al
call delay
mov al,0x18 
mov dx, TIMER0
out dx,al 
mov al,0x00
out dx,al
call delay
mov al,0x19 
mov dx, TIMER0
out dx,al 
mov al,0x00
out dx,al
call delay 
mov al,0x20 
mov dx, TIMER0
out dx,al 
mov al,0x00
out dx,al
call delay
mov al,0x21 
mov dx, TIMER0
out dx,al 
mov al,0x00
out dx,al
call delay
mov al,0x22 
mov dx, TIMER0
out dx,al 
mov al,0x00
out dx,al
call delay   
mov al,0x00 
mov dx, TIMER_CONTROL
out dx,al  
call lcd_cls
jmp again
valid_pass: 
call lcd_cls
lea bx,msg7
call lcd_puts
Not_Set:
call getk
cmp al,0xA
jnz not_np 
call lcd_cls
lea bx,msg9
call lcd_puts
call new_pass
jmp again
not_np:
cmp al,0xB
jnz Not_Set
call lcd_cls
lea bx,msg8
call lcd_puts
mov al,0x80
out portb,al
call delay
call delay 
call delay
call delay
call delay
call delay
call delay
call delay
call delay
call delay
call delay
call delay
call delay
mov al,0x00
out portb,al
jmp again 
jmp $


getk proc
push bx
push cx
push dx
mov bx,0x2f
mov dx,0x0   

COL1:
mov al,0x6
out PORTC,al 
call delay
in al,PORTC
and al,0xf0
cmp al,0xf0
jnz COL1_D

COL2:
mov al,0x5
out PORTC,al
call delay
in al,PORTC
and al,0xf0
cmp al,0xf0
jnz COL2_D

COL3:
mov al,0x3
out PORTC,al
call delay
in al,PORTC
and al,0xf0
cmp al,0xf0
jnz COL3_D

jmp COL1

COL1_D:
shl al,1
inc dx
jc COL1_D 
dec dx
mov al,0x3
mul dl
add al,0x3
mov cl,al
lea bx,KEYPAD_DATA
add bl,cl
dec bx
mov al,[bx]
jmp end_gk

COL2_D:
shl al,1
inc dx
jc COL2_D
dec dx
mov al,0x3
mul dl
add al,0x2
mov cl,al
lea bx,KEYPAD_DATA
add bl,cl
dec bx
mov al,[bx]
jmp end_gk

COL3_D:
shl al,1
inc dx
jc COL3_D
dec dx
mov al,0x3
mul dl
add al,0x1
mov cl,al
lea bx,KEYPAD_DATA
add bl,cl
dec bx
mov al,[bx]
end_gk:
pop dx
pop cx
pop bx    
push ax 
;mov al,0x88
;out PORT_CON,al  
;mov ax,0xFF 
pop ax
ret
getk endp



get_pass proc
    push ax
    push bx
    push cx 
    mov cx,0
    lea bx,tpassword  
gnc:
    push bx    
    call getk  
    cmp al,0xA
    jnz not_clear
    call lcd_l2
    lea bx,msg4
    call lcd_puts 
    call lcd_puts 
    call lcd_puts 
    call lcd_l2
    call lcd_puts
    mov cx,0
    pop bx
    lea bx,tpassword 
    jmp gnc
not_clear:
    cmp al,0xB
    jz  gnc
    call lcd_printd
    mov bx,0xFF
    call delay 
    call delay
    call delay 
    call delay
    pop bx 
    mov BYTE PTR [BX] , al
    inc BX 
    inc cx
    cmp cX,0x6
    jnz gnc 
    pop cx
    pop bx
    pop ax 
    ret 
get_pass endp 

new_pass proc
    push ax
    push bx
    push cx 
    mov cx,0
    lea bx,password  
gnn:
    push bx    
    call getk  
    cmp al,0xA
    jnz not_clearn
    call lcd_l2
    lea bx,msg4
    call lcd_puts 
    call lcd_puts 
    call lcd_puts 
    call lcd_l2
    call lcd_puts
    mov cx,0
    pop bx
    lea bx,password 
    jmp gnn
not_clearn:
    cmp al,0xB
    jz  gnc
    call lcd_printd
    mov bx,0xFF
    call delay 
    call delay
    call delay 
    call delay
    pop bx 
    mov BYTE PTR [BX] , al
    inc BX 
    inc cx
    cmp cX,0x6
    jnz gnc 
    pop cx
    pop bx
    pop ax 
    ret 
new_pass endp 

check_pass proc
    push cx
    push bx
    lea bx,tpassword
    mov vara,bx
    lea bx,password
    mov varb,bx
    mov cx,0
cnp:
    mov bx,vara
    mov al,[BX]
    mov bx,varb
    mov ah,[BX]
    cmp al,ah
    jnz wrong_pass
    inc vara
    inc varb
    inc cx 
    cmp cx,0x6
    jnz cnp
    mov ax,0xFF
    pop bx
    pop cx 
    ret
wrong_pass:
    mov ax,0x0
    pop bx
    pop cx 
    ret    
check_pass endp    

delay proc
push ax
push bx
mov ax,0xff
repeat:
dec ax
nop
nop
nop
nop
nop
jnz repeat
mov ax,0xff
dec bx
jnz repeat
pop bx
pop ax
ret
delay endp        


lcd_init proc
    mov bx,0x05 
	mov ax,0
	out portb,ax
	call delay
	mov ax,0x2
	out porta,ax	
	out portb,ax
	call delay
	mov ax,0x0
	out portb,ax
	mov	ax,0x28
	call LCD_SEND_COMMAND
	mov	ax,0x06
	call LCD_SEND_COMMAND
	mov	ax,0x0C
	call LCD_SEND_COMMAND
	mov	ax,0x01
	call LCD_SEND_COMMAND  
	ret
lcd_init endp         


lcd_home proc
	push ax
	mov ax,0x80  
	mov bx,0x05 	
	call   LCD_SEND_COMMAND
    call delay
	pop	ax
	ret
lcd_home endp 

lcd_cls proc
	push ax
	mov	ax,0x01
	mov bx,0x05 
	call	LCD_SEND_COMMAND
	call   delay
	pop	ax
	ret
lcd_cls endp

lcd_l2 proc
    push bx
    push ax 
	mov	ax,0xc0
	mov bx,0x05 
	call	LCD_SEND_COMMAND
	call   delay
	pop ax
	pop bx
	ret
lcd_l2 endp 



lcd_putc proc 
    push bx
    push ax 
    mov bx,0x05
	call	LCD_SEND_DATA
	call	delay
	pop ax
	pop bx
	ret
lcd_putc endp

LCD_SEND_COMMAND proc 
    mov bx,0x05
    push ax
	mov varc,al
	mov vard,al
	and	varc,0xF0
	and	vard,0x0F
	  
	mov ax,0
	out portb,ax
	
    call delay 
    
	shr varc,4 
	
	mov al,varc
	out	porta,al
	
	mov ax,0x2
	out portb,ax 
	
	call delay  
	
	mov ax,0x0
	out portb,ax 
	
	call delay 
	
	mov al,vard
	out	porta,al 
	  
	mov ax,0x2
	out portb,ax  
	
	call delay   
	
	mov ax,0x0
	out portb,ax
	out	porta,ax 
	
	POP	ax
	RET 
LCD_SEND_COMMAND endp


LCD_SEND_DATA proc
    mov bx,0x05  
    push ax
	mov varc,al
	mov vard,al
	and	varc,0xF0
	and	vard,0x0F  
	mov ax,0
	out portb,ax
    call delay
	mov ax,0x1
	out portb,ax
	shr varc,4
	mov al,varc
	out	porta,al
	mov ax,0x3
	out portb,ax
	call delay
	mov ax,0x1
	out portb,ax
	call delay
	mov al,vard
	out	porta,al 
	mov ax,0x3
	out portb,ax
	call delay
	mov ax,0x01
	out portb,ax
	mov ax,0x00
	out porta,ax	
	out porta,ax
	POP	ax
	RET 
LCD_SEND_DATA endp




lcd_puts proc 
    push ax
    push bx
NC:
    mov al,[bx]
    cmp al,0
    jz  end_p
    cmp al,0x10
    jnz NE
    call lcd_l2
    inc bx
    jmp NC
NE:       
    call lcd_putc 
    inc bx
    jmp NC
end_p:        
    pop bx
    pop ax
    ret       
lcd_puts endp



lcd_printd proc
    push ax
    push cx 
    push dx      
    mov dl,0xa
ND:       
    div dl
    mov cl,al
    mov al,ah
    add al,'0'
    call lcd_putc
    pop dx
    pop cx
    pop ax
    ret
lcd_printd endp    
    
    