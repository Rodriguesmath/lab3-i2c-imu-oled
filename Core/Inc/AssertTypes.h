/******************************************************************************/
/**
 * @file    AssertTypes.h
 * @brief   Definicao de tipos primitivos padronizados de dados.
 * @author  Rodrigues
 * @note    Padrao de tipos baseado nas diretrizes de boas praticas do 
 *          Laboratorio ASSERT (IFPB).
 * @{
 ******************************************************************************/
#ifndef _ASSERT_TYPES_H_
#define _ASSERT_TYPES_H_

#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************
 * TIPOS DE DADOS PUBLICOS
 ******************************************************************************/
typedef uint8_t   u8;
typedef uint16_t  u16;
typedef uint32_t  u32;
typedef int8_t    s8;
typedef int16_t   s16;
typedef int32_t   s32;

#ifndef dTRUE
#define dTRUE     true
#endif

#ifndef dFALSE
#define dFALSE    false
#endif

#endif /* _ASSERT_TYPES_H_ */
/** @} DOXYGEN GROUP TAG END OF FILE */
